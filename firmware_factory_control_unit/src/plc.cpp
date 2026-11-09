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
const char* store_problem = "";  // why downloads are refused ("": they are not)
std::uint32_t received = 0;      // bytes of the running download in image_ram
bool downloading = false;

// ---- registers ------------------------------------------------------------------------------------------
/// Handshake registers the firmware answers in place (docs/plc-iec61131.md 6.3): never cyclic outputs.
constexpr bool is_command(std::uint16_t reg) noexcept {
    return reg == rm::holding::stepper1_start_homing || reg == rm::holding::stepper2_start_homing ||
           reg == rm::holding::ro_arm_mission_control;
}

/// The outputs that go to 0 in the safe state (STOP, ERROR, switching the source). Stepper targets,
/// speeds and accelerations stay as they are: the safe state of an axis is "no new target".
constexpr std::array<std::uint16_t, 9> safe_zero{
    rm::holding::valve1,       rm::holding::valve2,         rm::holding::valve3,
    rm::holding::vacuum_valve, rm::holding::vacuum_pump,    rm::holding::conveyor_pwm,
    rm::holding::compressor_pwm, rm::holding::ws2812_ch1_pattern, rm::holding::ws2812_ch2_pattern};

void safe_state() noexcept {
    for (const std::uint16_t r : safe_zero) registers.set_holding(r, 0);
}

/// The register bank's write filter: while the program is the control source, only the command
/// registers stay writable for Modbus, OPC UA and the web UI. Runs with the bank's guard held.
bool permit(void*, std::uint16_t start, std::uint16_t count) noexcept {
    if (!iec.load(std::memory_order_relaxed)) return true;
    for (std::uint32_t r = start; r < std::uint32_t{start} + count; ++r) {
        if (!is_command(static_cast<std::uint16_t>(r))) return false;
    }
    return true;
}

/// The image's I/O bindings against this firmware's register map; nullptr: fine.
const char* check_io() noexcept {
    for (std::size_t i = 0; i < machine.io_count(); ++i) {
        const vm::IoBinding b = machine.io(i);
        const unsigned words = b.bits == 32 ? 2 : 1;
        const std::size_t limit = b.area == vm::IoArea::input ? rm::input_count : rm::holding_count;
        if (b.reg + words > limit) return "the program binds a register this firmware does not have";
        if (b.kind == vm::IoKind::output) {
            if (b.area != vm::IoArea::holding) return "the program binds an output to an input register";
            for (unsigned w = 0; w < words; ++w) {
                if (is_command(static_cast<std::uint16_t>(b.reg + w))) return "the program binds a command register as an output";
            }
        } else if (b.kind != vm::IoKind::input) {
            return "the program uses register classes this firmware does not support yet";
        }
    }
    return nullptr;
}

void copy_inputs() noexcept {
    std::array<std::uint16_t, rm::input_count> in{};
    std::array<std::uint16_t, rm::holding_count> hold{};
    registers.read_inputs(0, in);
    registers.read_holdings(0, hold);
    const std::span<std::uint8_t> d = machine.data();
    for (std::size_t i = 0; i < machine.io_count(); ++i) {
        const vm::IoBinding b = machine.io(i);
        if (b.kind != vm::IoKind::input) continue;
        const std::uint16_t* t = b.area == vm::IoArea::input ? in.data() : hold.data();
        if (b.bits == 1) {
            d[b.addr] = t[b.reg] != 0 ? 1 : 0;
        } else if (b.bits == 16) {
            std::memcpy(&d[b.addr], &t[b.reg], 2);
        } else {
            const std::uint32_t v = (std::uint32_t{t[b.reg]} << 16) | t[b.reg + 1];  // high word first
            std::memcpy(&d[b.addr], &v, 4);
        }
    }
}

void copy_outputs() noexcept {
    if (!iec.load(std::memory_order_relaxed)) return;
    const std::span<const std::uint8_t> d = machine.data();
    for (std::size_t i = 0; i < machine.io_count(); ++i) {
        const vm::IoBinding b = machine.io(i);
        if (b.kind != vm::IoKind::output) continue;
        if (b.bits == 1) {
            registers.set_holding(b.reg, d[b.addr] != 0 ? 1 : 0);
        } else if (b.bits == 16) {
            std::uint16_t v;
            std::memcpy(&v, &d[b.addr], 2);
            registers.set_holding(b.reg, v);
        } else {
            std::uint32_t v;
            std::memcpy(&v, &d[b.addr], 4);
            const std::array<std::uint16_t, 2> words{static_cast<std::uint16_t>(v >> 16), static_cast<std::uint16_t>(v)};
            registers.set_holdings(b.reg, words);  // one guard: never half a value
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
    }
    return "?";
}

void enter(ws::PlcState s) noexcept {
    const ws::PlcState old = state.exchange(s);
    if (old == s) return;
    if (s != ws::PlcState::Run && iec.load()) safe_state();
    if (s == ws::PlcState::Run) max_instructions = 0;
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

/// STM32H563 boards of the old firmware kept the RoArm missions in littlefs in the same area. Until
/// they are moved onto the SD card (src/roarm/missions.cpp), the PLC must not overwrite them.
void check_store_area() noexcept {
    store_problem = "";
#if RTOS_FCU_REVISION != 3
    if (store.has_slot()) return;
    const auto magic_at = [](std::uint32_t block) {
        return std::memcmp(reinterpret_cast<const void*>(slots_base + block * SlotFlash::block_bytes + 8), "littlefs", 8) == 0;
    };
    if (magic_at(0) || magic_at(1)) {
        store_problem = "the flash still holds RoArm missions of the old firmware: insert an SD card and restart (they are moved there)";
        RTOS_LOG_WARN("plc: %s", store_problem);
    }
#endif
}

void set_source(bool to_iec) noexcept {
    if (to_iec == iec.load()) return;
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
    if (*store_problem != '\0') return store_problem;
    return store.set_config(config()) == vm::SlotError::none ? nullptr : "the flash could not be written";
}

// ---- requests from the network thread -------------------------------------------------------------------
enum class Kind : std::uint8_t { control, upload, commit, write };

struct Request {
    SessionId session{};
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

const char* control(ws::PlcCommand c) noexcept {
    using C = ws::PlcCommand;
    const ws::PlcState s = state.load();
    switch (c) {
        case C::Run:
            if (s == ws::PlcState::Run) return nullptr;
            if (s != ws::PlcState::Stop) return s == ws::PlcState::Error ? "the PLC is in ERROR: reset first" : "no program loaded";
            enter(ws::PlcState::Run);
            return nullptr;
        case C::Stop:
            if (s == ws::PlcState::Run) enter(ws::PlcState::Stop);
            return nullptr;
        case C::ResetWarm:
        case C::ResetCold:  // the same until RETAIN exists
            return load_ram_image();
        case C::SourceIec:
        case C::SourceFieldbus:
            // the switch must survive a restart: nothing changes when it cannot be stored
            if (*store_problem != '\0') return store_problem;
            set_source(c == C::SourceIec);
            return persist();
        case C::AutostartOn:
        case C::AutostartOff:
            if (*store_problem != '\0') return store_problem;
            autostart = c == C::AutostartOn;
            return persist();
    }
    return "unknown command";
}

const char* upload(const Request& r) noexcept {
    if (r.offset == 0) {
        if (state.load() == ws::PlcState::Run) return "stop the PLC first";
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
    if (*store_problem != '\0') {
        load_from_flash();
        return store_problem;
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
    switch (r.kind) {
        case Kind::control: {
            const char* e = control(r.command);
            out(ws::ControlResponse::Encode({.requestId = r.request_id, .ok = e == nullptr, .message = e ? e : ""}, buf.data(), buf.size()));
            break;
        }
        case Kind::upload: {
            const char* e = upload(r);
            out(ws::UploadResponse::Encode({.requestId = r.request_id, .ok = e == nullptr, .message = e ? e : ""}, buf.data(), buf.size()));
            break;
        }
        case Kind::commit: {
            const char* e = commit(r);
            out(ws::CommitResponse::Encode({.requestId = r.request_id, .ok = e == nullptr, .message = e ? e : ""}, buf.data(), buf.size()));
            break;
        }
        case Kind::write: {
            const bool ok = machine.loaded() && r.address + r.length <= machine.data_size();
            if (ok) std::memcpy(&machine.data()[r.address], r.bytes.data(), r.length);
            out(ws::WriteResponse::Encode({.requestId = r.request_id, .ok = ok}, buf.data(), buf.size()));
            break;
        }
    }
}

// ---- the thread -------------------------------------------------------------------------------------------
void run_cycle() noexcept {
    copy_inputs();
    const vm::RunResult r = machine.run_cycle(0, kernel.now());
    cycles.fetch_add(1, std::memory_order_relaxed);
    last_instructions = r.instructions;
    if (r.instructions > max_instructions) max_instructions = r.instructions;
    if (r.state == vm::RunState::trapped) {
        trap = static_cast<std::uint8_t>(r.trap);
        trap_code = r.trap_code;
        trap_pc = r.pc;
        RTOS_LOG_ERROR("plc: runtime error %u at pc %u: ERROR, outputs to the safe state", static_cast<unsigned>(r.trap),
                       static_cast<unsigned>(r.pc));
        enter(ws::PlcState::Error);
        return;
    }
    copy_outputs();
}

void plc_main(void*) {
    store.mount();
    check_store_area();
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
        if ((*bits & bit_cycle) != 0 && state.load() == ws::PlcState::Run) run_cycle();
    }
}

void post(const Request& r) noexcept {
    if (!requests.try_send(r)) {
        RTOS_LOG_WARN("plc: request queue full, request dropped");
        return;
    }
    kernel.notify(plc_tcb, notify_slot, bit_request);
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

bool external_write_allowed(std::uint16_t reg) noexcept { return !iec_controls() || is_command(reg); }

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
            .maxImageBytes = static_cast<std::uint32_t>(image_ram.size()), .dataBytes = static_cast<std::uint32_t>(data.size()),
            .storeProblem = store_problem};
        std::array<std::uint8_t, ws::StatusResponse::StatusResponse_MAX_SIZE> buf{};
        send(std::span(buf).first(ws::StatusResponse::Encode(p, buf.data(), buf.size())));
    } else if (type == ws::ReadRequest::TYPE_ID) {
        ws::ReadRequest::Payload q{};
        if (!ws::ReadRequest::Decode(d, len, q)) return;
        // the values of the last cycles (a value may be read half-updated while a cycle runs)
        const bool ok = machine.loaded() && q.length <= 1024 && std::uint64_t{q.address} + q.length <= machine.data_size();
        const ws::ReadResponse::Payload p{.requestId = q.requestId, .ok = ok, .address = q.address,
                                          .dataData = ok ? &data[q.address] : nullptr, .dataCount = ok ? q.length : 0u};
        std::array<std::uint8_t, ws::ReadResponse::ReadResponse_MAX_SIZE> buf{};
        send(std::span(buf).first(ws::ReadResponse::Encode(p, buf.data(), buf.size())));
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
    }
}

}  // namespace fcu::plc
