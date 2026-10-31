// Factory Control Unit firmware on rtoos.
//
// Boot (I/O thread): reset cause, identity check, banner -> supply gate (INA226, USB-PD) ->
// power consumers (pins, buses, sensors, drivers) -> network thread -> I/O cycle every 50 ms.
#include <algorithm>
#include <array>

#include "board/fcu.hpp"
#include "boot.hpp"
#include "io.hpp"
#include "network.hpp"
#include "rtos/port/cm33/cycle_counter.hpp"
#include "system.hpp"
#include "task_monitor.hpp"

fcu::Kernel kernel;
RTOS_CM33_BIND_KERNEL(kernel)
fcu::Registers registers;
fcu::Journal journal;

namespace {

namespace cm33 = rtos::port::cm33;
using fcu::Kernel;

// ---- console: the journal to USART3 ------------------------------------------------------------
Kernel::Thread log_tcb;
alignas(8) std::array<std::byte, 2048> log_stack;

void log_main(void*) {
    rtos::log::Cursor cursor = journal.oldest();
    std::array<char, 120> text{};
    std::array<char, 200> line{};
    for (;;) {
        rtos::log::Record record;
        std::uint32_t lost = 0;
        while (journal.read(cursor, record, text, lost)) {
            if (lost != 0) {
                board::print("... lines lost\r\n");
            }
            const std::size_t n = rtos::log::format_line(record, line, false);
            board::print({line.data(), n});
        }
        kernel.sleep(20);
    }
}

// ---- I/O thread: boot, then the cycle ------------------------------------------------------------
Kernel::Thread io_tcb;
// deepest path: the servo and stepper transactions, the mission store (littlefs) and a
// 1.3 KB request copied out of the RoArm queue
alignas(8) std::array<std::byte, 8 * 1024> io_stack;

void io_main(void*) {
    fcu::boot::announce();
    if (!fcu::boot::identity_matches()) {
        RTOS_LOG_ERROR("this firmware was provisioned for another board (chip UID): run the builder for this one");
        for (;;) {
            board::led_info_toggle();
            kernel.sleep(100);
        }
    }
    if (!fcu::boot::wait_for_supply()) {
        RTOS_LOG_ERROR("supply unknown: starting the power consumers anyway");
    }
    fcu::io::setup();
    fcu::net::start();
    RTOS_LOG_INFO("I/O cycle every %u ms", static_cast<unsigned>(fcu::io::cycle_ms));
    rtos::Tick next = kernel.now();
    for (;;) {
        fcu::io::cycle(kernel.now());
        next += fcu::io::cycle_ms;
        const rtos::Tick now = kernel.now();
        kernel.sleep(static_cast<std::int32_t>(next - now) > 0 ? next - now : 1);
        if (static_cast<std::int32_t>(now - next) > static_cast<std::int32_t>(fcu::io::cycle_ms)) {
            next = now;  // overrun: no catching up
        }
    }
}

}  // namespace

int main() {
    fcu::boot::read_reset_cause();
    board::init();
    cm33::CycleCounter::enable();
    fcu::Port::configure(board::sysclk_hz, 1000);
    rtos::log::set_sink(journal.sink([]() noexcept { return kernel.now(); }));
    std::ranges::fill(log_stack, fcu::task_monitor::paint);
    std::ranges::fill(io_stack, fcu::task_monitor::paint);
    kernel.create_thread(log_tcb, {.name = "log", .entry = &log_main, .stack = log_stack, .priority = fcu::log_priority});
    kernel.create_thread(io_tcb, {.name = "io", .entry = &io_main, .stack = io_stack, .priority = fcu::io_priority});
    fcu::task_monitor::add(io_tcb, io_stack);
    fcu::task_monitor::add(log_tcb, log_stack);
    kernel.start();
}
