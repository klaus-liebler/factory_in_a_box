/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TOF3_IRQ_Pin GPIO_PIN_13
#define TOF3_IRQ_GPIO_Port GPIOC
#define I1D_Pin GPIO_PIN_0
#define I1D_GPIO_Port GPIOF
#define USB_VSENSE_Pin GPIO_PIN_1
#define USB_VSENSE_GPIO_Port GPIOF
#define TOF2_IRQ_Pin GPIO_PIN_0
#define TOF2_IRQ_GPIO_Port GPIOA
#define TOF1_IRQ_Pin GPIO_PIN_1
#define TOF1_IRQ_GPIO_Port GPIOA
#define STM32BOOT_TX_Pin GPIO_PIN_2
#define STM32BOOT_TX_GPIO_Port GPIOA
#define STM32BOOT_RX_Pin GPIO_PIN_3
#define STM32BOOT_RX_GPIO_Port GPIOA
#define I1C_Pin GPIO_PIN_4
#define I1C_GPIO_Port GPIOC
#define STEPPER1_DIR_Pin GPIO_PIN_5
#define STEPPER1_DIR_GPIO_Port GPIOC
#define STEPPER1_STEP_Pin GPIO_PIN_0
#define STEPPER1_STEP_GPIO_Port GPIOB
#define STEPPER2_STEP_Pin GPIO_PIN_1
#define STEPPER2_STEP_GPIO_Port GPIOB
#define STEPPER2_DIR_Pin GPIO_PIN_2
#define STEPPER2_DIR_GPIO_Port GPIOB
#define STEPPER_TX_Pin GPIO_PIN_10
#define STEPPER_TX_GPIO_Port GPIOB
#define STEPPER_RX_Pin GPIO_PIN_11
#define STEPPER_RX_GPIO_Port GPIOB
#define I2C_Pin GPIO_PIN_8
#define I2C_GPIO_Port GPIOC
#define I2D_Pin GPIO_PIN_9
#define I2D_GPIO_Port GPIOC
#define STEPPER_EN_Pin GPIO_PIN_8
#define STEPPER_EN_GPIO_Port GPIOA
#define I3C_Pin GPIO_PIN_15
#define I3C_GPIO_Port GPIOA
#define IO4_Pin GPIO_PIN_10
#define IO4_GPIO_Port GPIOC
#define HX711_DATA_Pin GPIO_PIN_11
#define HX711_DATA_GPIO_Port GPIOC
#define HX711_CLK_Pin GPIO_PIN_12
#define HX711_CLK_GPIO_Port GPIOC
#define IO3_Pin GPIO_PIN_2
#define IO3_GPIO_Port GPIOD
#define IO2_Pin GPIO_PIN_5
#define IO2_GPIO_Port GPIOB
#define I3D_Pin GPIO_PIN_7
#define I3D_GPIO_Port GPIOB
#define IO1_Pin GPIO_PIN_9
#define IO1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
