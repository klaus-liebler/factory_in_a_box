// The Malibotti robot (ESP32-S3) on USART6, FCU revision 3 (J16). The robot keeps its missions
// itself; the FCU starts them through the register MalibottiMissionControl (as RoArmMissionControl)
// and shows the robot's state in MalibottiStatus / MalibottiActiveMission. The link: framed
// BestBinaryBuffers messages (schema/best_binary_buffers/malibotti.cs, docs/malibotti-uart.md).
// On revision 1 these are empty: the registers stay 0.
#pragma once

#include <cstdint>

namespace fcu::malibotti {

/// The UART and its interrupt (the I/O thread's setup).
void setup() noexcept;
/// One I/O cycle: the received frames, the control register, a command to (re)send, the status registers.
void cycle(std::uint32_t now) noexcept;

}  // namespace fcu::malibotti
