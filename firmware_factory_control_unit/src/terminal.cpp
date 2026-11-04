// Browser terminal (see terminal.hpp).
#include "terminal.hpp"

#include <array>
#include <charconv>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string_view>

#include "board/fcu.hpp"
#include "device_ids.hh"
#include "gitconstants.hh"
#include "hardware.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/sync/message_queue.hpp"
#include "system.hpp"
#include "terminal_commands.hh"
#include "ws_protocol.hh"

namespace fcu::terminal {
namespace {

namespace h5 = rtos::port::stm32h5;
namespace rm = register_map;
using h5::gpio::GpioPort;
using h5::gpio::Pin;

namespace ws = WsProtocol::terminal;
namespace cat = terminal_catalog;

constexpr std::size_t max_message =
    std::max(ws::CommandResponse::CommandResponse_MAX_SIZE, ws::LookupResponse::LookupResponse_MAX_SIZE);
constexpr std::size_t max_output =
    std::min(ws::CommandResponse::CommandResponse_MAX_SIZE, ws::LookupResponse::LookupResponse_MAX_SIZE) - 16;

/// The output of a command: lines of text.
class Out {
public:
    [[gnu::format(printf, 2, 3)]] void line(const char* format, ...) noexcept {
        va_list args;
        va_start(args, format);
        const int n = std::vsnprintf(text_.data() + size_, text_.size() - size_, format, args);
        va_end(args);
        if (n > 0) {
            size_ = std::min(size_ + static_cast<std::size_t>(n), text_.size() - 1);
        }
        if (size_ + 1 < text_.size()) {
            text_[size_++] = '\n';
        }
        text_[size_] = '\0';
    }
    [[nodiscard]] const char* c_str() noexcept {
        if (size_ > 0 && text_[size_ - 1] == '\n') {
            text_[--size_] = '\0';
        }
        return text_.data();
    }

private:
    std::array<char, max_output + 1> text_{};
    std::size_t size_ = 0;
};

using Args = std::span<const std::string_view>;
struct Handler {
    cat::Command id;
    bool (*run)(Args, Out&);  ///< false: wrong arguments (the usage line is shown)
};

// ---- helpers --------------------------------------------------------------------------------------

template <class T>
std::optional<T> number(std::string_view s) {
    T v{};
    int base = 10;
    if (s.starts_with("0x") || s.starts_with("0X")) {
        s.remove_prefix(2);
        base = 16;
    }
    const auto [end, ec] = std::from_chars(s.data(), s.data() + s.size(), v, base);
    if (ec != std::errc{} || end != s.data() + s.size()) {
        return std::nullopt;
    }
    return v;
}

/// "PC7", "pa15": a pin of the LQFP100 package.
std::optional<Pin> parse_pin(std::string_view s) {
    if (s.size() < 3 || (s[0] != 'P' && s[0] != 'p')) {
        return std::nullopt;
    }
    const char port = static_cast<char>(s[1] & ~0x20);
    const auto n = number<unsigned>(s.substr(2));
    if (port < 'A' || port > 'I' || !n || *n > 15) {
        return std::nullopt;
    }
    const Pin p{static_cast<GpioPort>(port - 'A'), static_cast<std::uint8_t>(*n)};
    for (const auto& [pp, pn] : h5::gpio::package_pins) {
        if (pp == p.port && pn == p.number) {
            return p;
        }
    }
    return std::nullopt;
}

std::optional<bool> parse_level(std::string_view s) {
    if (s == "1" || s == "on" || s == "high") return true;
    if (s == "0" || s == "off" || s == "low") return false;
    return std::nullopt;
}

char port_letter(Pin p) { return static_cast<char>('A' + static_cast<int>(p.port)); }

// ---- named inputs and outputs ------------------------------------------------------------------------
struct NamedPin {
    std::string_view name;
    Pin pin;
    bool active_low;
};

std::span<const NamedPin> inputs() {
    static constexpr auto common = std::array{
        NamedPin{"light_barrier1", pins::light_barrier1, true}, NamedPin{"light_barrier2", pins::light_barrier2, true},
        NamedPin{"tof1_irq", pins::tof1_irq, true},             NamedPin{"tof2_irq", pins::tof2_irq, true},
        NamedPin{"tof3_irq", pins::tof3_irq, true},             NamedPin{"color_irq", pins::color_irq, true},
        NamedPin{"usb_vsense", pins::usb_vsense, false},        NamedPin{"current_alert", pins::current_alert, true},
        NamedPin{"hx711_dout", pins::hx711_dout, false},
    };
    // light_barrier3 exists on revision 1 only (pins.hpp: std::optional)
    static constexpr auto all = [] {
        std::array<NamedPin, common.size() + (pins::light_barrier3 ? 1 : 0)> a{};
        for (std::size_t i = 0; i < common.size(); ++i) a[i] = common[i];
        if (pins::light_barrier3) a[common.size()] = {"light_barrier3", *pins::light_barrier3, true};
        return a;
    }();
    return all;
}

struct NamedOutput {
    std::string_view name;
    std::uint16_t holding;  ///< the register the I/O cycle drives the pin from
    bool exists;
};
constexpr std::array outputs{
    NamedOutput{"valve1", rm::holding::valve1, true},
    NamedOutput{"valve2", rm::holding::valve2, true},
    NamedOutput{"valve3", rm::holding::valve3, true},
    NamedOutput{"vacuum_valve", rm::holding::vacuum_valve, !pins::rev1},
    NamedOutput{"vacuum_pump", rm::holding::vacuum_pump, !pins::rev1},
    NamedOutput{"stepper_enable", rm::holding::stepper_enable, true},
};

// ---- commands (schema/terminal/commands.json) -------------------------------------------------------------
bool cmd_info(Args, Out& o) {
    o.line("%s.local, hardware revision %u", DEVICE_HOSTNAME, board::revision);
    o.line("firmware %.*s (%.*s)", static_cast<int>(git::VERSION.size()), git::VERSION.data(),
           static_cast<int>(git::COMMIT_HASH.size()), git::COMMIT_HASH.data());
    const auto s = net::status();
    const unsigned a = s.ethernet_address.value;
    o.line("uptime %lu s, ethernet %s %u.%u.%u.%u, %u Mbit/s", static_cast<unsigned long>(kernel.now() / 1000),
           s.ethernet_link ? "up" : "down", a >> 24, (a >> 16) & 0xFF, (a >> 8) & 0xFF, a & 0xFF, s.ethernet_mbps);
    return true;
}

bool cmd_in(Args a, Out& o) {
    if (a.size() > 1) return false;
    bool found = false;
    for (const NamedPin& p : inputs()) {
        if (a.empty() || a[0] == p.name) {
            const bool level = h5::gpio::read(p.pin);
            o.line("%-15s P%c%-2u %s%s", p.name.data(), port_letter(p.pin), p.pin.number, level ? "high" : "low",
                   level != p.active_low ? "  (active)" : "");
            found = true;
        }
    }
    return found;
}

bool cmd_out(Args a, Out& o) {
    if (a.empty()) {
        for (const NamedOutput& out : outputs) {
            if (out.exists) {
                o.line("%-15s %s", out.name.data(), registers.holding(out.holding) != 0 ? "on" : "off");
            }
        }
        return true;
    }
    if (a.size() != 2) return false;
    const auto level = parse_level(a[1]);
    for (const NamedOutput& out : outputs) {
        if (out.exists && out.name == a[0] && level) {
            registers.set_holding(out.holding, *level ? 1 : 0);  // the I/O cycle drives the pin
            o.line("%s %s", out.name.data(), *level ? "on" : "off");
            return true;
        }
    }
    return false;
}

bool cmd_pwm(Args a, Out& o) {
    const auto permille = a.size() == 2 ? number<unsigned>(a[1]) : std::nullopt;
    if (!permille || *permille > 1000 || (a[0] != "conveyor" && a[0] != "compressor")) return false;
    registers.set_holding(a[0] == "conveyor" ? rm::holding::conveyor_pwm : rm::holding::compressor_pwm,
                          static_cast<std::uint16_t>(*permille));
    o.line("%.*s %u.%u %%", static_cast<int>(a[0].size()), a[0].data(), *permille / 10, *permille % 10);
    return true;
}

bool cmd_gpio(Args a, Out& o) {
    const auto pin = a.empty() ? std::nullopt : parse_pin(a[0]);
    if (!pin || a.size() > 2) return false;
    if (a.size() == 2) {
        const auto level = parse_level(a[1]);
        if (!level) return false;
        h5::gpio::write(*pin, *level);
        o.line("P%c%u <- %s (an output pin of the I/O cycle is overwritten by it within 50 ms)", port_letter(*pin),
               pin->number, *level ? "high" : "low");
    }
    o.line("P%c%u = %s", port_letter(*pin), pin->number, h5::gpio::read(*pin) ? "high" : "low");
    return true;
}

bool cmd_adc(Args, Out& o) {
    o.line("pressure sensor (ADC1 INP18): %u of 4095", hw::Pressure::read());
    return true;
}

bool cmd_power(Args, Out& o) {
    if (const auto m = hw::Supply::read()) {
        o.line("bus %ld mV, shunt %ld uV, current %ld mA, power %lu mW", static_cast<long>(m->bus_mv),
               static_cast<long>(m->shunt_uv), static_cast<long>(m->current_ma), static_cast<unsigned long>(m->power_mw));
        return true;
    }
    o.line("INA226 does not answer");
    return true;
}

bool cmd_temp(Args, Out& o) {
    o.line("CPU %d C", rm::cpu_temperature_c(registers));
    return true;
}

bool cmd_sensors(Args, Out& o) {
    o.line("ToF 1/2/3: %u / %u / %u mm (status %u %u %u)", rm::tof1_distance_mm(registers), rm::tof2_distance_mm(registers),
           rm::tof3_distance_mm(registers), rm::tof1_status(registers), rm::tof2_status(registers), rm::tof3_status(registers));
    o.line("colour C/R/G/B: %u %u %u %u", rm::color_clear(registers), rm::color_red(registers),
           rm::color_green(registers), rm::color_blue(registers));
    o.line("scale A: %ld (status %u)", static_cast<long>(rm::scale_a_weight(registers)), rm::scale_a_status(registers));
    return true;
}

template <class Bus>
void scan(unsigned bus, Out& o) {
    unsigned found = 0;
    for (std::uint8_t addr = 0x08; addr < 0x78; ++addr) {
        if (Bus::probe(addr)) {
            o.line("I2C%u: device at 0x%02X", bus, addr);
            ++found;
        }
    }
    o.line("I2C%u: %u device(s)", bus, found);
}

bool cmd_i2c(Args a, Out& o) {
    const auto bus = a.size() == 1 ? number<unsigned>(a[0]) : std::nullopt;
    if (!bus) return false;
    switch (*bus) {
        case 1: scan<hw::I2c1>(1, o); return true;
        case 2: scan<hw::I2c2>(2, o); return true;
        case 4: scan<hw::I2c4>(4, o); return true;
        default: return false;
    }
}

bool cmd_reg(Args a, Out& o) {
    if (a.size() < 2 || (a[0] != "h" && a[0] != "i")) return false;
    const bool holding = a[0] == "h";
    const auto addr = number<unsigned>(a[1]);
    const std::size_t count = holding ? register_map::holding_count : register_map::input_count;
    if (!addr || *addr >= count) return false;
    if (a.size() == 3) {
        const auto value = number<int>(a[2]);
        if (!holding || !value || *value < -32768 || *value > 65535) return false;
        registers.set_holding(static_cast<std::uint16_t>(*addr), static_cast<std::uint16_t>(*value));
    }
    const std::uint16_t v = holding ? registers.holding(static_cast<std::uint16_t>(*addr))
                                    : registers.input(static_cast<std::uint16_t>(*addr));
    o.line("%s %u = %u (0x%04X, signed %d)", holding ? "holding" : "input", *addr, v, v, static_cast<std::int16_t>(v));
    return true;
}

bool cmd_stepper(Args a, Out& o) {
    const auto n = a.size() == 2 ? number<unsigned>(a[0]) : std::nullopt;
    const auto target = a.size() == 2 ? number<std::int32_t>(a[1]) : std::nullopt;
    if (!n || (*n != 1 && *n != 2) || !target) return false;
    rm::set_stepper_enable(registers, 1);
    if (*n == 1) {
        rm::set_stepper1_target(registers, *target);
    } else {
        rm::set_stepper2_target(registers, *target);
    }
    o.line("stepper %u: enabled, target %ld (position now %ld)", *n, static_cast<long>(*target),
           static_cast<long>(*n == 1 ? rm::stepper1_position(registers) : rm::stepper2_position(registers)));
    return true;
}

bool cmd_led(Args a, Out& o) {
    const auto ch = a.size() == 2 ? number<unsigned>(a[0]) : std::nullopt;
    const auto pattern = a.size() == 2 ? number<unsigned>(a[1]) : std::nullopt;
    if (!ch || (*ch != 1 && *ch != 2) || !pattern || *pattern > 0xFFFF) return false;
    registers.set_holding(*ch == 1 ? rm::holding::ws2812_ch1_pattern : rm::holding::ws2812_ch2_pattern,
                          static_cast<std::uint16_t>(*pattern));
    o.line("WS2812 channel %u: pattern %u", *ch, *pattern);
    return true;
}

bool cmd_reset(Args a, Out& o) {
    if (a.size() != 1 || a[0] != "now") return false;
    o.line("resetting");
    RTOS_LOG_WARN("terminal: reset requested");
    kernel.sleep(200);  // the answer and the log line go out first
    Port::system_reset();
}

constexpr std::array handlers{
    Handler{cat::Command::info, &cmd_info},       Handler{cat::Command::in, &cmd_in},
    Handler{cat::Command::out, &cmd_out},         Handler{cat::Command::pwm, &cmd_pwm},
    Handler{cat::Command::gpio, &cmd_gpio},       Handler{cat::Command::adc, &cmd_adc},
    Handler{cat::Command::power, &cmd_power},     Handler{cat::Command::temp, &cmd_temp},
    Handler{cat::Command::sensors, &cmd_sensors}, Handler{cat::Command::i2c, &cmd_i2c},
    Handler{cat::Command::reg, &cmd_reg},         Handler{cat::Command::stepper, &cmd_stepper},
    Handler{cat::Command::led, &cmd_led},         Handler{cat::Command::reset, &cmd_reset},
};

/// Every command of the catalog has exactly one handler.
consteval bool handlers_complete() {
    for (const cat::CommandInfo& c : cat::commands) {
        std::size_t n = 0;
        for (const Handler& h : handlers) n += h.id == c.id ? 1 : 0;
        if (n != 1) return false;
    }
    return handlers.size() == cat::commands.size();
}
static_assert(handlers_complete(), "schema/terminal/commands.json and the handlers in terminal.cpp differ");

// ---- lookups (schema/terminal/commands.json): one "value \t description" line per value ----------------
void lookup(cat::Lookup l, Out& o) {
    switch (l) {  // no default: -Wswitch reports a lookup without an answer
        case cat::Lookup::inputs:
            for (const NamedPin& p : inputs()) {
                const bool level = h5::gpio::read(p.pin);
                o.line("%s\tP%c%u %s%s", p.name.data(), port_letter(p.pin), p.pin.number, level ? "high" : "low",
                       level != p.active_low ? " (active)" : "");
            }
            return;
        case cat::Lookup::outputs:
            for (const NamedOutput& out : outputs) {
                if (out.exists) {
                    o.line("%s\t%s", out.name.data(), registers.holding(out.holding) != 0 ? "on" : "off");
                }
            }
            return;
        case cat::Lookup::pins:
            for (const auto& [port, number] : h5::gpio::package_pins) {
                const Pin p{port, number};
                o.line("P%c%u\t%s", port_letter(p), p.number, h5::gpio::read(p) ? "high" : "low");
            }
            return;
    }
}

// ---- queue: network thread -> I/O thread --------------------------------------------------------------
struct Request {
    net::Server::SessionId session = 0;
    std::uint16_t request_id = 0;
    std::optional<cat::Lookup> lookup;  ///< a lookup instead of a command line
    std::array<char, 201> line{};
};
rtos::MessageQueue<Kernel, Request, 4> requests{kernel};

void reply(const Request& r, std::span<const std::uint8_t> message) {
    if (!message.empty()) {
        (void)net::outbox.send(r.session, std::as_bytes(message));
    }
}

void run(const Request& r) {
    Out out;
    std::array<std::uint8_t, max_message> buf{};
    if (r.lookup) {
        lookup(*r.lookup, out);
        const ws::LookupResponse::Payload p{.requestId = r.request_id, .ok = true, .values = out.c_str()};
        reply(r, std::span(buf).first(ws::LookupResponse::Encode(p, buf.data(), buf.size())));
        return;
    }
    std::array<std::string_view, 8> words{};
    std::size_t n = 0;
    std::string_view rest(r.line.data());
    while (!rest.empty() && n < words.size()) {
        const auto start = rest.find_first_not_of(' ');
        if (start == std::string_view::npos) break;
        rest.remove_prefix(start);
        const auto end = rest.find(' ');
        words[n++] = rest.substr(0, end);
        rest = end == std::string_view::npos ? std::string_view{} : rest.substr(end);
    }
    bool ok = true;
    if (n > 0) {
        const cat::CommandInfo* c = nullptr;
        for (const cat::CommandInfo& info : cat::commands) {
            if (info.name == words[0]) c = &info;
        }
        const Handler* h = nullptr;
        for (const Handler& handler : handlers) {
            if (c != nullptr && handler.id == c->id) h = &handler;
        }
        if (h == nullptr) {
            out.line("unknown command '%.*s' - try help", static_cast<int>(words[0].size()), words[0].data());
            ok = false;
        } else if (!h->run(std::span(words).subspan(1, n - 1), out)) {
            out.line("usage: %.*s", static_cast<int>(c->usage.size()), c->usage.data());
            ok = false;
        }
    }
    const ws::CommandResponse::Payload p{.requestId = r.request_id, .ok = ok, .output = out.c_str()};
    reply(r, std::span(buf).first(ws::CommandResponse::Encode(p, buf.data(), buf.size())));
}

}  // namespace

void post(net::Server::SessionId id, std::uint16_t request_id, std::span<const char> line) noexcept {
    Request r{.session = id, .request_id = request_id, .lookup = std::nullopt};
    std::memcpy(r.line.data(), line.data(), std::min(line.size(), r.line.size() - 1));
    if (!requests.try_send(r)) {
        RTOS_LOG_WARN("terminal: command queue full, command dropped");
    }
}

bool post_lookup(net::Server::SessionId id, std::uint16_t request_id, std::uint8_t lookup) noexcept {
    if (lookup >= cat::lookup_count) {
        return false;
    }
    if (!requests.try_send({.session = id, .request_id = request_id, .lookup = static_cast<cat::Lookup>(lookup)})) {
        RTOS_LOG_WARN("terminal: command queue full, lookup dropped");
    }
    return true;
}

void cycle() noexcept {
    while (const auto r = requests.try_receive()) {
        run(*r);
    }
}

}  // namespace fcu::terminal
