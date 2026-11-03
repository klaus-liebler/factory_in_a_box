// System-wide objects of the FCU firmware: kernel, register bank, log journal.
//
// Threads (0 = most urgent):
//   net   8   network thread: IP stack, HTTPS/WebSocket, Modbus TCP/RTU, OPC UA, USB, mDNS,
//             DHCP, SNTP (rtos::net::NetRuntime)
//   io   10   boot sequence (supply voltage, USB-PD, power consumers), then the I/O cycle
//             every 50 ms: sensors, actuators, steppers, RoArm, heartbeat
//   log  14   console: writes the log journal to USART3
#pragma once

#include <cstddef>
#include <cstdint>

#include "register_map.hpp"
#include "rtos/log/log.hpp"
#include "rtos/modbus/register_bank.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/rtos.hpp"

namespace fcu {

struct KernelConfig : rtos::DefaultConfig {
    static constexpr std::size_t max_threads = 5;
    static constexpr std::size_t notify_slots = 2;
};

using Port = rtos::port::cm33::Cm33Port;
using Kernel = rtos::Kernel<Port, KernelConfig>;

/// The Modbus/OPC UA register map (register_map_schema/*.cs, generated register_map.hpp):
/// shared by the network thread (protocols) and the I/O thread (values).
using Registers = rtos::modbus::RegisterBank<register_map::holding_count, register_map::input_count, Kernel::Lock>;

/// A small buffer only: records go to the console and to every browser right away; the
/// browser keeps the history (Terminal page).
using Journal = rtos::log::Journal<48, 120, Kernel::Lock>;

inline constexpr std::uint8_t net_priority = 8;
inline constexpr std::uint8_t io_priority = 10;
inline constexpr std::uint8_t log_priority = 14;

/// NVIC priorities (0..15). 5 and above are kernel-aware (Kernel::Lock masks them); 0..4 are
/// never masked and must not call the kernel.
inline constexpr std::uint8_t irq_priority_motion = 4; ///< stepper DMA refill (deadline: one step), no kernel calls
inline constexpr std::uint8_t irq_priority_net = 6;    ///< ETH, USB
inline constexpr std::uint8_t irq_priority_pd = 7;     ///< UCPD and its deadline timer (same level)

}  // namespace fcu

extern fcu::Kernel kernel;
extern fcu::Registers registers;
extern fcu::Journal journal;
