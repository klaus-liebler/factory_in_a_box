// The IEC 61131-3 PLC runtime (see plc.hpp).
#include "plc.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <span>
#include <string_view>

#include "board/chip.hpp"
#include "rtos/plc/slot_store.hpp"
#include "rtos/plc/vm.hpp"
#include "rtos/sync/message_queue.hpp"
#include "system.hpp"
#include "task_monitor.hpp"
#include "ws_protocol.hh"
#if RTOS_FCU_REVISION == 3
#include "rtos/port/stm32h7/flash_program.hpp"
#else
#include "rtos/port/stm32h5/flash_program.hpp"
#endif

namespace fcu::plc {
namespace {

namespace vm = rtos::plc;
namespace ws = WsProtocol::plc;
namespace rm = register_map;
using SessionId = net::Server::SessionId;

// ---- memory and flash -------------------------------------------------------------------------------
// The program slots: 256 KB at the end of bank 2 on both chips (FLASH_DATA_START in CMakeLists.txt).
constexpr std::uintptr_t slots_base = 0x081C'0000;
using SlotFlash = board::chip::flash::Region<slots_base, 256 * 1024, board::sysclk_hz>;
SlotFlash slot_flash;
vm::SlotStore<SlotFlash> store{slot_flash};

constexpr std::size_t image_capacity = 96 * 1024;  // docs/plc-iec61131.md 4.4
static_assert(image_capacity <= decltype(store)::max_image_bytes);
std::array<std::uint8_t, image_capacity> image_ram;  // the loaded image, and the download's buffer
std::array<std::uint8_t, 32 * 1024> data;           // the program's data segment
std::array<vm::Context, 1> contexts;
vm::Vm machine{data, contexts};

// ---- state (the PLC thread writes, the network thread reads) -----------------------------------------
std::atomic<ws::PlcState> state{ws::PlcState::NoProgram};
std::atomic<bool> iec{false};  // the control source is the program
bool autostart = false;
std::uint32_t image_bytes = 0;  // of the loaded image in image_ram
std::atomic<std::uint32_t> cycles{0}, last_instructions{0}, max_instructions{0};
std::atomic<std::uint8_t> trap{0};
std::atomic<std::uint16_t> trap_code{0};
std::atomic<std::uint32_t> trap_pc{0};
std::uint32_t received = 0;      // bytes of the running download in image_ram
bool downloading = false;

// ---- registers ------------------------------------------------------------------------------------------
/// The register classes come from the schema (Kind = Output | Command | Parameter, docs/plc-iec61131.md
/// 6.3): outputs are written every cycle, commands (handshakes the firmware answers in the register)
/// only when the program changed them, parameters stay writable from outside.
using rm::RegisterKind;
constexpr bool all_of_kind(std::uint16_t reg, unsigned words, RegisterKind k) noexcept {
    for (unsigned w = 0; w < words; ++w) {
        if (rm::holding_kind(static_cast<std::uint16_t>(reg + w)) != k) return false;
    }
    return true;
}

/// The command variables as the cycle found them (copy_inputs), in the order of the bindings:
/// copy_outputs writes a command only if the program changed it - never the answer back over itself.
constexpr std::size_t max_commands = 16;
std::array<std::uint32_t, max_commands> command_before{};

/// The outputs that go to 0 in the safe state (STOP, ERROR, switching the source). Stepper targets,
/// speeds and accelerations stay as they are: the safe state of an axis is "no new target".
constexpr std::array<std::uint16_t, 9> safe_zero{
    rm::holding::valve1,       rm::holding::valve2,         rm::holding::valve3,
    rm::holding::vacuum_valve, rm::holding::vacuum_pump,    rm::holding::conveyor_pwm,
    rm::holding::compressor_pwm, rm::holding::ws2812_ch1_pattern, rm::holding::ws2812_ch2_pattern};

void safe_state() noexcept {
    for (const std::uint16_t r : safe_zero) registers.set_holding(r, 0);
}

/// The register bank's write filter: while the program is the control source, only the parameter
/// registers stay writable for Modbus, OPC UA and the web UI. Runs with the bank's guard held.
bool permit(void*, std::uint16_t start, std::uint16_t count) noexcept {
    if (!iec.load(std::memory_order_relaxed)) return true;
    return all_of_kind(start, count, RegisterKind::parameter);
}

/// The image's I/O bindings against this firmware's register map; nullptr: fine.
const char* check_io() noexcept {
    if (machine.io_map_hash() != rm::map_hash) {
        return "the program was built for another register map: recompile the project";
    }
    std::size_t commands = 0;
    for (std::size_t i = 0; i < machine.io_count(); ++i) {
        const vm::IoBinding b = machine.io(i);
        const unsigned words = b.bits == 32 ? 2 : 1;
        const std::size_t limit = b.area == vm::IoArea::input ? rm::input_count : rm::holding_count;
        if (b.reg + words > limit) return "the program binds a register this firmware does not have";
        switch (b.kind) {
            case vm::IoKind::input: break;
            case vm::IoKind::output:
                if (b.area != vm::IoArea::holding || !all_of_kind(b.reg, words, RegisterKind::output)) {
                    return "the program binds an output to a register that is no output";
                }
                break;
            case vm::IoKind::command:
                if (b.area != vm::IoArea::holding || !all_of_kind(b.reg, words, RegisterKind::command)) {
                    return "the program binds a command to a register that is no command";
                }
                if (++commands > max_commands) return "the program binds too many command registers";
                break;
            case vm::IoKind::parameter:
                if (b.area != vm::IoArea::holding || !all_of_kind(b.reg, words, RegisterKind::parameter)) {
                    return "the program binds a parameter to a register that is no parameter";
                }
                break;
            default: return "the program uses an unknown register class";
        }
    }
    return nullptr;
}

std::uint32_t read_var(const vm::IoBinding& b) noexcept {
    const std::span<const std::uint8_t> d = machine.data();
    if (b.bits == 1) return d[b.addr];
    if (b.bits == 16) {
        std::uint16_t v;
        std::memcpy(&v, &d[b.addr], 2);
        return v;
    }
    std::uint32_t v;
    std::memcpy(&v, &d[b.addr], 4);
    return v;
}

void write_register(const vm::IoBinding& b, std::uint32_t v) noexcept {
    if (b.bits == 1) {
        registers.set_holding(b.reg, v != 0 ? 1 : 0);
    } else if (b.bits == 16) {
        registers.set_holding(b.reg, static_cast<std::uint16_t>(v));
    } else {
        const std::array<std::uint16_t, 2> words{static_cast<std::uint16_t>(v >> 16), static_cast<std::uint16_t>(v)};
        registers.set_holdings(b.reg, words);  // one guard: never half a value
    }
}

/// Registers -> program variables: inputs, parameters and the commands' answers.
void copy_inputs() noexcept {
    std::array<std::uint16_t, rm::input_count> in{};
    std::array<std::uint16_t, rm::holding_count> hold{};
    registers.read_inputs(0, in);
    registers.read_holdings(0, hold);
    const std::span<std::uint8_t> d = machine.data();
    std::size_t command = 0;
    for (std::size_t i = 0; i < machine.io_count(); ++i) {
        const vm::IoBinding b = machine.io(i);
        if (b.kind == vm::IoKind::output) continue;
        const std::uint16_t* t = b.area == vm::IoArea::input ? in.data() : hold.data();
        std::uint32_t v;
        if (b.bits == 1) {
            d[b.addr] = t[b.reg] != 0 ? 1 : 0;
            v = d[b.addr];
        } else if (b.bits == 16) {
            std::memcpy(&d[b.addr], &t[b.reg], 2);
            v = t[b.reg];
        } else {
            v = (std::uint32_t{t[b.reg]} << 16) | t[b.reg + 1];  // high word first
            std::memcpy(&d[b.addr], &v, 4);
        }
        if (b.kind == vm::IoKind::command) command_before[command++] = v;
    }
}

/// Program variables -> registers (control source IEC only): every output, a command only when the
/// program changed it in this cycle (a firmware answer that came meanwhile is not overwritten).
void copy_outputs() noexcept {
    if (!iec.load(std::memory_order_relaxed)) return;
    std::size_t command = 0;
    for (std::size_t i = 0; i < machine.io_count(); ++i) {
        const vm::IoBinding b = machine.io(i);
        if (b.kind == vm::IoKind::output) {
            write_register(b, read_var(b));
        } else if (b.kind == vm::IoKind::command) {
            const std::uint32_t v = read_var(b);
            if (v != command_before[command++]) write_register(b, v);
        }
    }
}

// ---- states ---------------------------------------------------------------------------------------------
const char* state_name(ws::PlcState s) noexcept {
    switch (s) {
        case ws::PlcState::NoProgram: return "NO_PROGRAM";
        case ws::PlcState::Stop: return "STOP";
        case ws::PlcState::Run: return "RUN";
        case ws::PlcState::Error: return "ERROR";
        case ws::PlcState::Halted: return "HALTED";
    }
    return "?";
}

// ---- forcing and the debug lease (docs/plc-iec61131.md 10.3) -------------------------------------------
/// A forced variable: its bytes go into the data segment after the inputs are read and again after
/// the program ran - an input force beats the input image, an output force beats the program.
struct Force {
    std::uint32_t address = 0;
    std::uint8_t length = 0;  ///< 0: free
    std::array<std::uint8_t, 8> bytes{};
};
std::array<Force, 32> forces{};  // the PLC thread only
std::atomic<std::uint8_t> force_count{0};
/// The session that forces holds the lease: the others may watch, not change. Ends with the last
/// force or when its WebSocket closes (the forces stay then, docs/plc-iec61131.md 10.3).
constexpr SessionId no_session = net::Outbox::all_sessions;  // never a client's id
std::atomic<SessionId> lease{no_session};

void apply_forces() noexcept {
    if (force_count.load(std::memory_order_relaxed) == 0) return;
    const std::span<std::uint8_t> d = machine.data();
    for (const Force& f : forces) {
        if (f.length != 0) std::memcpy(&d[f.address], f.bytes.data(), f.length);
    }
}

// ---- breakpoints and single step (docs/plc-iec61131.md 10.3, rtoos docs/plc.md "Debugging") -----------
std::array<std::uint32_t, 64> breakpoints{};  // sorted; the PLC thread only
std::size_t breakpoint_count = 0;
vm::StepKind step = vm::StepKind::none;
std::uint32_t step_depth = 0;
std::atomic<std::uint8_t> halt_reason{0};  ///< 1 breakpoint, 2 step (while HALTED)

void apply_debug() noexcept {
    machine.set_debug({.breakpoints = std::span(breakpoints).first(breakpoint_count), .step = step, .step_depth = step_depth});
}

/// Nobody debugs any more (no force, no breakpoint): the lease ends.
void lease_check() noexcept {
    if (force_count.load() == 0 && breakpoint_count == 0) lease = no_session;
}

void release_forces(const char* why) noexcept {
    if (const unsigned n = force_count.load(); n != 0) RTOS_LOG_INFO("plc: %u forced value(s) released (%s)", n, why);
    forces = {};
    force_count = 0;
    lease_check();
}

void clear_breakpoints(const char* why) noexcept {
    if (breakpoint_count != 0) RTOS_LOG_INFO("plc: %u breakpoint(s) cleared (%s)", static_cast<unsigned>(breakpoint_count), why);
    breakpoint_count = 0;
    step = vm::StepKind::none;
    apply_debug();
    lease_check();
}

void enter(ws::PlcState s) noexcept {
    const ws::PlcState old = state.exchange(s);
    if (old == s) return;
    // HALTED keeps everything as it is: the outputs keep their values, the forces stay
    if (s != ws::PlcState::Run && s != ws::PlcState::Halted) {
        if (old == ws::PlcState::Halted) machine.abandon(0);  // the halted cycle stays unfinished
        release_forces(state_name(s));
        clear_breakpoints(state_name(s));
        if (iec.load()) safe_state();
    }
    if (s == ws::PlcState::Run && old != ws::PlcState::Halted) max_instructions = 0;
    RTOS_LOG_INFO("plc: %s", state_name(s));
}

/// Loads image_ram[0, image_bytes) into the VM (initial values): STOP, or NO_PROGRAM with a reason.
const char* load_ram_image() noexcept {
    machine.unload();
    if (image_bytes == 0) {
        enter(ws::PlcState::NoProgram);
        return "no program";
    }
    if (const vm::LoadError e = machine.load(std::span(image_ram).first(image_bytes)); e != vm::LoadError::none) {
        enter(ws::PlcState::NoProgram);
        RTOS_LOG_ERROR("plc: the image is not loadable (%s)", vm::to_string(e));
        return vm::to_string(e);
    }
    if (const char* problem = check_io()) {
        machine.unload();
        enter(ws::PlcState::NoProgram);
        RTOS_LOG_ERROR("plc: %s", problem);
        return problem;
    }
    trap = 0;
    trap_code = 0;
    trap_pc = 0;
    cycles = 0;
    enter(ws::PlcState::Stop);
    return nullptr;
}

/// The program from the active flash slot into RAM and the VM.
void load_from_flash() noexcept {
    image_bytes = 0;
    if (store.has_image() && store.image_bytes() <= image_ram.size() &&
        store.read_image(std::span(image_ram).first(store.image_bytes())) == vm::SlotError::none) {
        image_bytes = store.image_bytes();
    }
    (void)load_ram_image();
}

void set_source(bool to_iec) noexcept {
    if (to_iec == iec.load()) return;
    release_forces("control source switched");
    if (to_iec) {
        iec = true;  // from now on the filter refuses external writes ...
        safe_state();  // ... and the outputs start from the safe state
    } else {
        if (state.load() == ws::PlcState::Run) enter(ws::PlcState::Stop);
        safe_state();
        iec = false;
    }
    RTOS_LOG_INFO("plc: control source %s", to_iec ? "IEC 61131" : "fieldbus (Modbus/OPC UA/web UI)");
}

vm::SlotConfig config() noexcept {
    return {.control_source = static_cast<std::uint8_t>(iec.load() ? 1 : 0), .autostart = static_cast<std::uint8_t>(autostart ? 1 : 0)};
}

const char* persist() noexcept {
    return store.set_config(config()) == vm::SlotError::none ? nullptr : "the flash could not be written";
}

// ---- requests from the network thread -------------------------------------------------------------------
enum class Kind : std::uint8_t { control, upload, commit, write, force, unforce, breakpoints, watch };

struct Request {
    SessionId session{};
    bool terminal = false;  ///< from the terminal: the result goes to the log, no response
    std::uint16_t request_id = 0;
    Kind kind = Kind::control;
    ws::PlcCommand command{};
    std::uint32_t total = 0, offset = 0, crc = 0, address = 0;
    std::uint16_t length = 0;
    std::array<std::uint8_t, 1024> bytes{};
};
rtos::MessageQueue<Kernel, Request, 2> requests{kernel};

Kernel::Thread plc_tcb;
alignas(8) std::array<std::byte, 6 * 1024> plc_stack;
constexpr std::size_t notify_slot = 0;
constexpr std::uint32_t bit_cycle = 1u << 0, bit_request = 1u << 1;

/// Sends an encoded response from the PLC thread (through the outbox).
void reply(SessionId session, std::span<const std::uint8_t> message) noexcept {
    if (!message.empty()) (void)net::outbox.send(session, std::as_bytes(message));
}

const char* debug_continue(vm::StepKind kind) noexcept;  // below, with the cycle

// ---- watch subscriptions (docs/plc-iec61131.md 10.3) -------------------------------------------------------
/// One browser's subscription: ranges of the data segment, pushed as WatchValues after a cycle, at most
/// every watch_period_ms. Owned by the PLC thread; a closed WebSocket only sets its bit in watch_closed
/// (session ids are reused: a new session must never get the old one's values).
constexpr std::size_t watch_ranges = 128, watch_bytes = 1024;
constexpr std::uint32_t watch_period_ms = 100;
struct Watch {
    bool active = false;
    std::uint16_t generation = 0;
    std::uint16_t count = 0;
    std::uint16_t total = 0;
    std::uint32_t last_ms = 0;
    std::array<std::uint32_t, watch_ranges> address{};
    std::array<std::uint16_t, watch_ranges> length{};
};
std::array<Watch, net::max_https_sessions> watches{};
std::atomic<std::uint32_t> watch_closed{0};
std::array<std::uint8_t, watch_bytes> watch_data{};
std::array<std::uint8_t, ws::WatchValues::WatchValues_MAX_SIZE> watch_message{};

/// Ends the subscriptions of the sessions closed since the last call.
void watch_drop_closed() noexcept {
    const std::uint32_t closed = watch_closed.exchange(0);
    for (std::size_t i = 0; i < watches.size(); ++i) {
        if ((closed & (1u << i)) != 0) watches[i].active = false;
    }
}

/// A WatchRequest (the ranges in r.bytes: count little-endian u32 addresses, then count u16 lengths).
const char* watch(const Request& r) noexcept {
    watch_drop_closed();  // a closed session's id may be this one's: drop the old subscription first
    if (r.session >= watches.size()) return "no such session";
    Watch& w = watches[r.session];
    const std::uint16_t count = r.length;
    if (count > watch_ranges) return "too many ranges (at most 128)";
    std::uint32_t total = 0;
    for (std::uint16_t i = 0; i < count; ++i) {
        std::memcpy(&w.address[i], &r.bytes[i * 4u], 4);
        std::memcpy(&w.length[i], &r.bytes[count * 4u + i * 2u], 2);
        total += w.length[i];
    }
    if (total > watch_bytes) {
        w.active = false;
        return "too many bytes (at most 1024)";
    }
    w.generation = static_cast<std::uint16_t>(r.address);
    w.count = count;
    w.total = static_cast<std::uint16_t>(total);
    w.active = count != 0;
    w.last_ms = kernel.now() - watch_period_ms;  // the first values with the next cycle
    return nullptr;
}

/// After a cycle (and in every state at the I/O rate): the values of each due subscription, all read
/// here between two cycles, so they belong to the same cycle. A range outside the data segment (another
/// program was loaded) sends no data: the browser subscribes anew for the new program.
void watch_publish() noexcept {
    watch_drop_closed();
    const std::uint32_t now = kernel.now();
    const std::size_t size = machine.loaded() ? machine.data_size() : 0;
    for (std::size_t i = 0; i < watches.size(); ++i) {
        Watch& w = watches[i];
        if (!w.active || static_cast<std::int32_t>(now - w.last_ms) < static_cast<std::int32_t>(watch_period_ms)) continue;
        w.last_ms = now;
        std::size_t n = 0;
        bool fits = true;
        for (std::uint16_t k = 0; k < w.count && fits; ++k) {
            fits = std::uint64_t{w.address[k]} + w.length[k] <= size;
            if (fits) {
                std::memcpy(&watch_data[n], &data[w.address[k]], w.length[k]);
                n += w.length[k];
            }
        }
        const ws::WatchValues::Payload p{.generation = w.generation, .cycles = cycles.load(), .state = state.load(),
                                         .dataData = watch_data.data(), .dataCount = fits ? static_cast<std::uint32_t>(n) : 0u};
        reply(static_cast<SessionId>(i), std::span(watch_message).first(ws::WatchValues::Encode(p, watch_message.data(), watch_message.size())));
    }
}

const char* control(ws::PlcCommand c) noexcept {
    using C = ws::PlcCommand;
    const ws::PlcState s = state.load();
    switch (c) {
        case C::Run:
            if (s == ws::PlcState::Run) return nullptr;
            if (s == ws::PlcState::Halted) return "the program is halted: continue or step";
            if (s != ws::PlcState::Stop) return s == ws::PlcState::Error ? "the PLC is in ERROR: reset first" : "no program loaded";
            enter(ws::PlcState::Run);
            return nullptr;
        case C::Stop:
            if (s == ws::PlcState::Run || s == ws::PlcState::Halted) enter(ws::PlcState::Stop);
            return nullptr;
        case C::Continue: return debug_continue(vm::StepKind::none);
        case C::StepInto: return debug_continue(vm::StepKind::into);
        case C::StepOver: return debug_continue(vm::StepKind::over);
        case C::StepOut: return debug_continue(vm::StepKind::out);
        case C::ResetWarm:
        case C::ResetCold:  // the same until RETAIN exists
            return load_ram_image();
        case C::SourceIec:
        case C::SourceFieldbus:
            set_source(c == C::SourceIec);
            return persist();
        case C::AutostartOn:
        case C::AutostartOff:
            autostart = c == C::AutostartOn;
            return persist();
        case C::UnforceAll:
            release_forces("released");
            return nullptr;
    }
    return "unknown command";
}

/// A force (the PLC thread): replaces one at the same address. nullptr: done.
const char* force(const Request& r) noexcept {
    if (!machine.loaded()) return "no program loaded";
    if (r.length == 0 || r.length > 8 || std::uint64_t{r.address} + r.length > machine.data_size()) {
        return "not a variable of the program";
    }
    Force* slot = nullptr;
    for (Force& f : forces) {
        if (f.length != 0 && f.address == r.address) slot = &f;
    }
    if (slot == nullptr) {
        for (Force& f : forces) {
            if (f.length == 0 && slot == nullptr) slot = &f;
        }
        if (slot == nullptr) return "too many forced values (32)";
        ++force_count;
    }
    slot->address = r.address;
    slot->length = static_cast<std::uint8_t>(r.length);
    std::memcpy(slot->bytes.data(), r.bytes.data(), r.length);
    lease = r.session;
    apply_forces();  // visible at once, also in STOP
    RTOS_LOG_WARN("plc: forced %u byte(s) at %lu (client %u)", static_cast<unsigned>(r.length),
                  static_cast<unsigned long>(r.address), static_cast<unsigned>(r.session));
    return nullptr;
}

const char* unforce(const Request& r) noexcept {
    if (r.address == 0xFFFF'FFFFu) {
        release_forces("released by the IDE");
        return nullptr;
    }
    for (Force& f : forces) {
        if (f.length != 0 && f.address == r.address) {
            f = Force{};
            --force_count;
            RTOS_LOG_INFO("plc: force at %lu released", static_cast<unsigned long>(r.address));
        }
    }
    lease_check();
    return nullptr;
}

/// The breakpoints (code offsets, r.length of them in r.bytes): replace the earlier ones. The VM checks
/// nothing about them - an offset that is no instruction start never matches.
const char* set_breakpoints(const Request& r) noexcept {
    if (!machine.loaded()) return "no program loaded";
    if (r.length > breakpoints.size()) return "too many breakpoints (64)";
    breakpoint_count = r.length;
    std::memcpy(breakpoints.data(), r.bytes.data(), r.length * 4u);
    std::sort(breakpoints.begin(), breakpoints.begin() + static_cast<std::ptrdiff_t>(breakpoint_count));
    if (breakpoint_count != 0) lease = r.session;
    apply_debug();
    lease_check();
    RTOS_LOG_INFO("plc: %u breakpoint(s) (client %u)", static_cast<unsigned>(breakpoint_count), static_cast<unsigned>(r.session));
    return nullptr;
}

const char* upload(const Request& r) noexcept {
    if (r.offset == 0) {
        if (state.load() == ws::PlcState::Run || state.load() == ws::PlcState::Halted) return "stop the PLC first";
        if (r.total == 0 || r.total > image_ram.size()) return "the image does not fit";
        machine.unload();  // image_ram becomes the download buffer
        enter(ws::PlcState::NoProgram);
        downloading = true;
        received = 0;
    }
    if (!downloading) return "no download running";
    if (r.offset != received || r.length == 0 || r.offset + r.length > r.total || r.total > image_ram.size()) {
        downloading = false;
        load_from_flash();
        return "chunk out of order";
    }
    std::memcpy(&image_ram[r.offset], r.bytes.data(), r.length);
    received += r.length;
    return nullptr;
}

const char* commit(const Request& r) noexcept {
    if (!downloading || received != r.total) return "the download is incomplete";
    downloading = false;
    const auto img = std::span<const std::uint8_t>(image_ram).first(received);
    if (vm::crc32(img) != r.crc) {
        load_from_flash();
        return "CRC mismatch: the image was damaged on the way";
    }
    image_bytes = received;
    if (const char* problem = load_ram_image()) {
        load_from_flash();  // keep the program that was there
        return problem;
    }
    if (store.commit(img, config()) != vm::SlotError::none) {
        load_from_flash();
        return "the flash could not be written";
    }
    RTOS_LOG_INFO("plc: program stored (%u bytes, slot %c)", static_cast<unsigned>(image_bytes), store.active_slot() == 0 ? 'A' : 'B');
    return nullptr;
}

void handle(const Request& r) noexcept {
    std::array<std::uint8_t, 256> buf{};  // the responses: a flag and a message of at most 120 bytes
    const auto out = [&](std::size_t n) { reply(r.session, std::span(buf).first(n)); };
    // the debug lease: another browser that forces may be watched, not overruled (the terminal may)
    const SessionId holder = lease.load();
    const char* leased = !r.terminal && holder != no_session && holder != r.session
                             ? "another browser is debugging this PLC (forces, breakpoints): release them there, or 'plc unforce' / STOP"
                             : nullptr;
    switch (r.kind) {
        case Kind::control: {
            const char* e = leased ? leased : control(r.command);
            if (r.terminal) {
                if (e != nullptr) RTOS_LOG_WARN("plc: terminal command refused: %s", e);
                else RTOS_LOG_INFO("plc: terminal command done (%s, control source %s, autostart %s)", state_name(state.load()),
                                   iec.load() ? "IEC 61131" : "fieldbus", autostart ? "on" : "off");
                break;
            }
            out(ws::ControlResponse::Encode({.requestId = r.request_id, .ok = e == nullptr, .message = e ? e : ""}, buf.data(), buf.size()));
            break;
        }
        case Kind::upload: {
            const char* e = leased ? leased : upload(r);
            out(ws::UploadResponse::Encode({.requestId = r.request_id, .ok = e == nullptr, .message = e ? e : ""}, buf.data(), buf.size()));
            break;
        }
        case Kind::commit: {
            const char* e = commit(r);
            out(ws::CommitResponse::Encode({.requestId = r.request_id, .ok = e == nullptr, .message = e ? e : ""}, buf.data(), buf.size()));
            break;
        }
        case Kind::write: {
            const bool ok = leased == nullptr && machine.loaded() && r.address + r.length <= machine.data_size();
            if (ok) std::memcpy(&machine.data()[r.address], r.bytes.data(), r.length);
            out(ws::WriteResponse::Encode({.requestId = r.request_id, .ok = ok}, buf.data(), buf.size()));
            break;
        }
        case Kind::force: {
            const char* e = leased ? leased : force(r);
            out(ws::ForceResponse::Encode({.requestId = r.request_id, .ok = e == nullptr, .message = e ? e : ""}, buf.data(), buf.size()));
            break;
        }
        case Kind::unforce: {
            const char* e = leased ? leased : unforce(r);
            out(ws::UnforceResponse::Encode({.requestId = r.request_id, .ok = e == nullptr, .message = e ? e : ""}, buf.data(), buf.size()));
            break;
        }
        case Kind::breakpoints: {
            const char* e = leased ? leased : set_breakpoints(r);
            out(ws::BreakpointsResponse::Encode({.requestId = r.request_id, .ok = e == nullptr, .message = e ? e : ""}, buf.data(), buf.size()));
            break;
        }
        case Kind::watch: {  // watching needs no lease: every browser may watch
            const char* e = watch(r);
            out(ws::WatchResponse::Encode({.requestId = r.request_id, .ok = e == nullptr, .message = e ? e : ""}, buf.data(), buf.size()));
            break;
        }
    }
}

// ---- the thread -------------------------------------------------------------------------------------------
/// The end of a cycle or of a piece of it (after a halt): halted again, a trap, or the outputs.
void finish(const vm::RunResult& r, bool fresh) noexcept {
    last_instructions = fresh ? r.instructions : last_instructions.load() + r.instructions;
    if (r.state == vm::RunState::halted) {
        // HALTED: the I/O cycle runs on, the outputs keep their values, nothing continues by itself
        halt_reason = step != vm::StepKind::none ? 2 : 1;
        if (state.load() != ws::PlcState::Halted) {
            RTOS_LOG_INFO("plc: halted at pc %lu (%s)", static_cast<unsigned long>(r.pc), halt_reason == 1 ? "breakpoint" : "step");
        }
        enter(ws::PlcState::Halted);
        return;
    }
    cycles.fetch_add(1, std::memory_order_relaxed);
    if (last_instructions > max_instructions) max_instructions = last_instructions.load();
    if (r.state == vm::RunState::trapped) {
        trap = static_cast<std::uint8_t>(r.trap);
        trap_code = r.trap_code;
        trap_pc = r.pc;
        RTOS_LOG_ERROR("plc: runtime error %u at pc %u: ERROR, outputs to the safe state", static_cast<unsigned>(r.trap),
                       static_cast<unsigned>(r.pc));
        enter(ws::PlcState::Error);
        return;
    }
    apply_forces();  // forced outputs beat the program
    copy_outputs();
    if (state.load() == ws::PlcState::Halted) enter(ws::PlcState::Run);
    if (step != vm::StepKind::none) {
        // a step past the end of the cycle halts at the first statement of the next one
        step = vm::StepKind::into;
        apply_debug();
    }
}

void run_cycle() noexcept {
    copy_inputs();
    apply_forces();  // forced inputs beat the input image
    finish(machine.run_cycle(0, kernel.now()), true);
}

/// HALTED: on to the next breakpoint (none) or one step; the rest of the cycle runs now.
const char* debug_continue(vm::StepKind kind) noexcept {
    if (state.load() != ws::PlcState::Halted || !machine.halted(0)) return "the program is not halted";
    step = kind;
    step_depth = machine.context(0).depth;
    apply_debug();
    finish(machine.resume(0), false);
    return nullptr;
}

void plc_main(void*) {
    store.mount();
    const vm::SlotConfig c = store.config();
    autostart = c.autostart != 0;
    registers.set_write_filter({nullptr, &permit});
    set_source(c.control_source != 0);
    load_from_flash();
    RTOS_LOG_INFO("plc: %s, control source %s, autostart %s", image_bytes ? "program loaded" : "no program",
                  iec.load() ? "IEC 61131" : "fieldbus", autostart ? "on" : "off");
    if (autostart && state.load() == ws::PlcState::Stop) enter(ws::PlcState::Run);
    for (;;) {
        const auto bits = kernel.notify_wait(notify_slot);
        if (!bits) continue;
        while (const auto r = requests.try_receive()) handle(*r);
        if ((*bits & bit_cycle) != 0) {
            if (state.load() == ws::PlcState::Run) run_cycle();
            watch_publish();
        }
    }
}

bool post(const Request& r) noexcept {
    if (!requests.try_send(r)) {
        RTOS_LOG_WARN("plc: request queue full, request dropped");
        return false;
    }
    kernel.notify(plc_tcb, notify_slot, bit_request);
    return true;
}

}  // namespace

// ---- interface ---------------------------------------------------------------------------------------------
void create_thread() noexcept {
    std::ranges::fill(plc_stack, task_monitor::paint);
    kernel.create_thread(plc_tcb, {.name = "plc", .entry = &plc_main, .stack = plc_stack, .priority = plc_priority});
    task_monitor::add(plc_tcb, plc_stack);
}

void io_cycle_done() noexcept { kernel.notify(plc_tcb, notify_slot, bit_cycle); }

bool iec_controls() noexcept { return iec.load(std::memory_order_relaxed); }

Status status() noexcept {
    const bool loaded = machine.loaded();
    return {.state = state_name(state.load()), .iec = iec.load(), .autostart = autostart,
            .image_bytes = loaded ? image_bytes : 0, .max_image_bytes = static_cast<std::uint32_t>(image_ram.size()),
            .cycles = cycles.load(), .last_instructions = last_instructions.load(), .max_instructions = max_instructions.load(),
            .trap = trap.load(), .trap_code = trap_code.load(), .trap_pc = trap_pc.load(),
            .forces = force_count.load()};
}

void session_closed(SessionId id) noexcept {
    if (id < 32) watch_closed.fetch_or(1u << id);  // its subscription ends (the PLC thread drops it)
    SessionId holder = id;
    if (lease.compare_exchange_strong(holder, no_session)) {
        RTOS_LOG_INFO("plc: the debug lease of client %u ended (its %u forced value(s) stay)", static_cast<unsigned>(id),
                      static_cast<unsigned>(force_count.load()));
    }
}

bool command(ws::PlcCommand c) noexcept { return post({.terminal = true, .kind = Kind::control, .command = c}); }

bool external_write_allowed(std::uint16_t reg) noexcept {
    return !iec_controls() || rm::holding_kind(reg) == RegisterKind::parameter;
}

void on_message(SessionId id, std::uint16_t type, const std::uint8_t* d, std::size_t len) noexcept {
    const auto send = [id](std::span<const std::uint8_t> b) {
        if (!b.empty()) (void)net::server.send(id, std::as_bytes(b));
    };
    if (type == ws::StatusRequest::TYPE_ID) {
        ws::StatusRequest::Payload q{};
        if (!ws::StatusRequest::Decode(d, len, q)) return;
        const ws::StatusResponse::Payload p{
            .requestId = q.requestId, .state = state.load(), .source = iec_controls() ? ws::ControlSource::Iec : ws::ControlSource::Fieldbus,
            .autostart = autostart, .cycleMs = 50, .cycles = cycles.load(), .lastInstructions = last_instructions.load(),
            .maxInstructions = max_instructions.load(), .imageBytes = machine.loaded() ? image_bytes : 0,
            .projectHash = machine.loaded() ? machine.project_hash() : 0, .ioMapHash = machine.loaded() ? machine.io_map_hash() : 0,
            .trap = trap.load(), .trapCode = trap_code.load(), .trapPc = trap_pc.load(),
            .maxImageBytes = static_cast<std::uint32_t>(image_ram.size()), .dataBytes = static_cast<std::uint32_t>(data.size()), .forces = force_count.load(),
            .leasedByOther = lease.load() != no_session && lease.load() != id};
        std::array<std::uint8_t, ws::StatusResponse::StatusResponse_MAX_SIZE> buf{};
        send(std::span(buf).first(ws::StatusResponse::Encode(p, buf.data(), buf.size())));
    } else if (type == ws::ControlRequest::TYPE_ID) {
        ws::ControlRequest::Payload q{};
        if (!ws::ControlRequest::Decode(d, len, q)) return;
        post({.session = id, .request_id = q.requestId, .kind = Kind::control, .command = q.command});
    } else if (type == ws::UploadRequest::TYPE_ID) {
        ws::UploadRequest::Payload q{};
        if (!ws::UploadRequest::Decode(d, len, q) || q.dataCount > 1024) return;
        Request r{.session = id, .request_id = q.requestId, .kind = Kind::upload, .total = q.totalBytes, .offset = q.offset,
                  .length = static_cast<std::uint16_t>(q.dataCount)};
        std::memcpy(r.bytes.data(), q.dataData, q.dataCount);
        post(r);
    } else if (type == ws::CommitRequest::TYPE_ID) {
        ws::CommitRequest::Payload q{};
        if (!ws::CommitRequest::Decode(d, len, q)) return;
        post({.session = id, .request_id = q.requestId, .kind = Kind::commit, .total = q.totalBytes, .crc = q.crc});
    } else if (type == ws::WriteRequest::TYPE_ID) {
        ws::WriteRequest::Payload q{};
        if (!ws::WriteRequest::Decode(d, len, q) || q.dataCount > 64) return;
        Request r{.session = id, .request_id = q.requestId, .kind = Kind::write, .address = q.address,
                  .length = static_cast<std::uint16_t>(q.dataCount)};
        std::memcpy(r.bytes.data(), q.dataData, q.dataCount);
        post(r);
    } else if (type == ws::ForceRequest::TYPE_ID) {
        ws::ForceRequest::Payload q{};
        if (!ws::ForceRequest::Decode(d, len, q) || q.dataCount > 8) return;
        Request r{.session = id, .request_id = q.requestId, .kind = Kind::force, .address = q.address,
                  .length = static_cast<std::uint16_t>(q.dataCount)};
        std::memcpy(r.bytes.data(), q.dataData, q.dataCount);
        post(r);
    } else if (type == ws::UnforceRequest::TYPE_ID) {
        ws::UnforceRequest::Payload q{};
        if (!ws::UnforceRequest::Decode(d, len, q)) return;
        post({.session = id, .request_id = q.requestId, .kind = Kind::unforce, .address = q.address});
    } else if (type == ws::WatchRequest::TYPE_ID) {
        ws::WatchRequest::Payload q{};
        if (!ws::WatchRequest::Decode(d, len, q) || q.addressesCount != q.lengthsCount || q.addressesCount > 128) return;
        Request r{.session = id, .request_id = q.requestId, .kind = Kind::watch, .address = q.generation,
                  .length = static_cast<std::uint16_t>(q.addressesCount)};
        std::memcpy(r.bytes.data(), q.addressesData, q.addressesCount * 4);  // little-endian, as the wire
        std::memcpy(r.bytes.data() + q.addressesCount * 4, q.lengthsData, q.lengthsCount * 2);
        post(r);
    } else if (type == ws::BreakpointsRequest::TYPE_ID) {
        ws::BreakpointsRequest::Payload q{};
        if (!ws::BreakpointsRequest::Decode(d, len, q) || q.offsetsCount > 64) return;
        Request r{.session = id, .request_id = q.requestId, .kind = Kind::breakpoints, .length = static_cast<std::uint16_t>(q.offsetsCount)};
        std::memcpy(r.bytes.data(), q.offsetsData, q.offsetsCount * 4);  // little-endian u32, as the VM's
        post(r);
    } else if (type == ws::HaltRequest::TYPE_ID) {
        ws::HaltRequest::Payload q{};
        if (!ws::HaltRequest::Decode(d, len, q)) return;
        // while HALTED the PLC thread does not touch the context (it waits for continue or a step)
        const bool halted = state.load() == ws::PlcState::Halted && machine.halted(0);
        std::array<std::uint32_t, vm::call_depth> returns{};
        std::uint32_t depth = 0;
        std::uint32_t pc = 0;
        if (halted) {
            const vm::Context& c = machine.context(0);
            depth = std::min<std::uint32_t>(c.depth, vm::call_depth);
            for (std::uint32_t i = 0; i < depth; ++i) returns[i] = c.calls[i].return_pc;
            pc = c.pc;
        }
        const ws::HaltResponse::Payload p{.requestId = q.requestId, .halted = halted, .reason = halted ? halt_reason.load() : std::uint8_t{0},
                                          .pc = pc, .returnPcsData = reinterpret_cast<const std::uint8_t*>(returns.data()),
                                          .returnPcsCount = depth};
        std::array<std::uint8_t, ws::HaltResponse::HaltResponse_MAX_SIZE> buf{};
        send(std::span(buf).first(ws::HaltResponse::Encode(p, buf.data(), buf.size())));
    }
}

}  // namespace fcu::plc
