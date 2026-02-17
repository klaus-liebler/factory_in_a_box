#pragma once

#include <cstdint>

struct TIM_TypeDef {
  uint32_t PSC = 0;
  uint32_t ARR = 0;
  uint32_t CNT = 0;
  uint32_t DIER = 0;
  uint32_t CR1 = 0;
  uint32_t CCR1 = 0;
};

inline TIM_TypeDef TIM17_Instance;

#define TIM17 (&TIM17_Instance)
#define TIM1_TRG_COM_TIM17_IRQn 0
#define TIM_DIER_UIE (1u << 0)
#define TIM_CR1_CEN (1u << 0)
