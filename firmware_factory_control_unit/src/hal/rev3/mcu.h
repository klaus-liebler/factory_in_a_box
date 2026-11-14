// MCU of FCU revision 3: STM32H743VIT6 (Cortex-M7, 400 MHz). The rtoos port headers of the
// peripherals the firmware uses, and the chip facts it reads directly (stm32h743xx.h, RM0433).
#pragma once

#include <cstdint>

#include "board/chip.hpp"
#include "rtos/port/stm32h7/adc.hpp"
#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/dma.hpp"
#include "rtos/port/stm32h7/exti.hpp"
#include "rtos/port/stm32h7/fdcan.hpp"
#include "rtos/port/stm32h7/flash_program.hpp"
#include "rtos/port/stm32h7/i2c.hpp"
#include "rtos/port/stm32h7/sdmmc.hpp"
#include "rtos/port/stm32h7/spi.hpp"
#include "rtos/port/stm32h7/timer.hpp"
#include "rtos/port/stm32h7/usart.hpp"

namespace fcu::mcu {

namespace chip = board::chip;

/// Reset cause: RCC_RSR (RCC_BASE 0x58024400, RSR at 0xD0, RCC_RSR_*_Pos).
using RCC_RSR = chip::Reg<0x5802'4400 + 0xD0>;
inline constexpr std::uint32_t rsr_rmvf = 1u << 16, rsr_borrstf = 1u << 21, rsr_pinrstf = 1u << 22,
                               rsr_sftrstf = 1u << 24, rsr_iwdgrstf = 1u << 26, rsr_wwdgrstf = 1u << 28,
                               rsr_lpwrrstf = 1u << 30;
/// The 96-bit unique ID (stm32h743xx.h UID_BASE).
inline constexpr std::uintptr_t uid_base = 0x1FF1'E800;

}  // namespace fcu::mcu
