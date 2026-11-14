// MCU of FCU revision 1 (first boards): STM32H563VIT6 (Cortex-M33, 250 MHz). The rtoos port
// headers of the peripherals the firmware uses - with the UCPD, revision 1's USB-PD sink - and
// the chip facts it reads directly (RM0481, SVD v1.8).
#pragma once

#include <cstdint>

#include "board/chip.hpp"
#include "rtos/port/stm32h5/adc.hpp"
#include "rtos/port/stm32h5/dts.hpp"
#include "rtos/port/stm32h5/exti.hpp"
#include "rtos/port/stm32h5/fdcan.hpp"
#include "rtos/port/stm32h5/flash_program.hpp"
#include "rtos/port/stm32h5/gpdma.hpp"
#include "rtos/port/stm32h5/i2c.hpp"
#include "rtos/port/stm32h5/sdmmc.hpp"
#include "rtos/port/stm32h5/spi.hpp"
#include "rtos/port/stm32h5/timer.hpp"
#include "rtos/port/stm32h5/ucpd.hpp"
#include "rtos/port/stm32h5/usart.hpp"

namespace fcu::mcu {

namespace chip = board::chip;

/// Reset cause: RCC_RSR (RCC base 0x44020C00 + 0xF4) and its flags.
using RCC_RSR = chip::PReg<0x0402'0C00 + 0xF4>;
inline constexpr std::uint32_t rsr_rmvf = 1u << 23, rsr_pinrstf = 1u << 26, rsr_borrstf = 1u << 27,
                               rsr_sftrstf = 1u << 28, rsr_iwdgrstf = 1u << 29, rsr_wwdgrstf = 1u << 30,
                               rsr_lpwrrstf = 1u << 31;
/// The 96-bit unique ID (RM0481 UID_BASE).
inline constexpr std::uintptr_t uid_base = 0x08FF'F800;

}  // namespace fcu::mcu
