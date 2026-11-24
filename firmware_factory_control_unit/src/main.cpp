// Factory Control Unit firmware on rtoos.
//
// Boot (I/O thread): reset cause, identity check, banner -> supply gate (INA226, USB-PD) ->
// power consumers (pins, buses, sensors, drivers) -> network thread -> I/O cycle every 50 ms.
#include <algorithm>
#include <array>

#include "auth.hpp"
#include "board/chip.hpp"
#include "board/fcu.hpp"
#include "boot.hpp"
#include "io.hpp"
#include "network.hpp"
#include "plc.hpp"
#include "steppers/steppers.hpp"
#include "rtos/time.hpp"
#include "system.hpp"
#include "task_monitor.hpp"

fcu::Kernel kernel;
RTOS_BOARD_BIND_KERNEL(kernel)
fcu::Registers registers{rtos::modbus::WriteHook{nullptr, &fcu::io::holding_written}};
fcu::Journal journal;
rtos::Semaphore<fcu::Kernel> flash_lock{kernel, 1, 1};

namespace {

using fcu::Kernel;

// ---- console: the journal to USART3 ------------------------------------------------------------
Kernel::Thread log_tcb;
// + the web accounts' save: CredentialStore builds two flash records (2 x 0.6 KB) on the stack
alignas(8) std::array<std::byte, 3584> log_stack;

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
        fcu::auth::store_pending_changes();  // between two log passes: may block for a flash erase
        kernel.sleep(20);
    }
}

// ---- I/O thread: boot, then the cycle ------------------------------------------------------------
Kernel::Thread io_tcb;
// deepest path: the servo and stepper transactions, the mission store (FileX) and a
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
    fcu::io::set_event_thread(io_tcb);
    const bool supply_up = fcu::boot::wait_for_supply();
    if (!supply_up) {
        RTOS_LOG_ERROR("supply unknown: starting the power consumers anyway");
    }
    fcu::io::setup(supply_up);
    fcu::net::start();
    RTOS_LOG_INFO("I/O cycle every %u ms", static_cast<unsigned>(fcu::io::cycle_ms));
    rtos::Tick next = kernel.now();
    for (;;) {
        fcu::io::cycle(kernel.now());
        fcu::plc::io_cycle_done();  // the inputs are fresh: one cycle of the IEC program
        next += fcu::io::cycle_ms;
        if (rtos::earlier(next + fcu::io::cycle_ms, kernel.now())) {
            next = kernel.now();  // overrun: no catching up
        }
        // until the next cycle: events from interrupts (a measurement ready, ...) right away, and
        // while a stepper homes its StallGuard poll every few ms
        for (rtos::Tick now = kernel.now(); rtos::earlier(now, next); now = kernel.now()) {
            const std::uint32_t poll = fcu::steppers::homing_poll_ms();
            const rtos::Tick wait = poll != 0 ? std::min<rtos::Tick>(poll, next - now) : next - now;
            if (const auto bits = kernel.notify_wait(fcu::io::event_slot, wait)) {
                fcu::io::events(*bits);
            }
            if (poll != 0) {
                fcu::steppers::homing_poll(kernel.now());
            }
        }
    }
}

}  // namespace

int main() {
    fcu::boot::read_reset_cause();
    board::init();
    board::CycleCounter::enable();
    fcu::Port::configure(board::sysclk_hz, 1000);
    rtos::log::set_sink(journal.sink([]() noexcept { return kernel.now(); }));
    fcu::task_monitor::spawn(log_tcb, {.name = "log", .entry = &log_main, .stack = log_stack, .priority = fcu::log_priority});
    fcu::task_monitor::spawn(io_tcb, {.name = "io", .entry = &io_main, .stack = io_stack, .priority = fcu::io_priority});
    fcu::plc::create_thread();
    kernel.start();
}
