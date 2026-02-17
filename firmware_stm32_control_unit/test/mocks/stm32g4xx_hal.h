#pragma once

#include <cstdint>

#define __HAL_RCC_TIM17_CLK_ENABLE() ((void)0)

inline void NVIC_SetPriority(int, uint32_t) {}
inline void NVIC_EnableIRQ(int) {}
