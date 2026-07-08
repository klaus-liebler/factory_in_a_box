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
#include "stm32h5xx_hal.h"

#include "stm32h5xx_ll_ucpd.h"
#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_cortex.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_ll_system.h"
#include "stm32h5xx_ll_utils.h"
#include "stm32h5xx_ll_pwr.h"
#include "stm32h5xx_ll_gpio.h"
#include "stm32h5xx_ll_dma.h"

#include "stm32h5xx_ll_exti.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SYNC1_Pin GPIO_PIN_3
#define SYNC1_GPIO_Port GPIOE
#define SYNC2_Pin GPIO_PIN_4
#define SYNC2_GPIO_Port GPIOE
#define WS2812_1_Pin GPIO_PIN_5
#define WS2812_1_GPIO_Port GPIOE
#define WS2812_2_Pin GPIO_PIN_6
#define WS2812_2_GPIO_Port GPIOE
#define PC13_Pin GPIO_PIN_13
#define PC13_GPIO_Port GPIOC
#define ETHERCAT_RESET_Pin GPIO_PIN_14
#define ETHERCAT_RESET_GPIO_Port GPIOC
#define TOF1_IRQ_Pin GPIO_PIN_15
#define TOF1_IRQ_GPIO_Port GPIOC
#define RGBSENS_IRQ_Pin GPIO_PIN_0
#define RGBSENS_IRQ_GPIO_Port GPIOH
#define TOF2_IRQ_Pin GPIO_PIN_1
#define TOF2_IRQ_GPIO_Port GPIOH
#define USB_VSENSE_Pin GPIO_PIN_0
#define USB_VSENSE_GPIO_Port GPIOC
#define HX711_SPI2_MISO_Pin GPIO_PIN_2
#define HX711_SPI2_MISO_GPIO_Port GPIOC
#define HX711_SPI2_MOSI_Pin GPIO_PIN_3
#define HX711_SPI2_MOSI_GPIO_Port GPIOC
#define ETH_RESET_Pin GPIO_PIN_0
#define ETH_RESET_GPIO_Port GPIOA
#define TOF3_IRQ_Pin GPIO_PIN_3
#define TOF3_IRQ_GPIO_Port GPIOA
#define PRESSURESENSOR_Pin GPIO_PIN_4
#define PRESSURESENSOR_GPIO_Port GPIOA
#define ETHERCAT_IO3_Pin GPIO_PIN_6
#define ETHERCAT_IO3_GPIO_Port GPIOA
#define ETHERCAT_IO1_Pin GPIO_PIN_0
#define ETHERCAT_IO1_GPIO_Port GPIOB
#define ETHERCAT_IO0_Pin GPIO_PIN_1
#define ETHERCAT_IO0_GPIO_Port GPIOB
#define ETHERCAT_CLK_Pin GPIO_PIN_2
#define ETHERCAT_CLK_GPIO_Port GPIOB
#define ROBOT_RX_Pin GPIO_PIN_7
#define ROBOT_RX_GPIO_Port GPIOE
#define ROBOT_TX_Pin GPIO_PIN_8
#define ROBOT_TX_GPIO_Port GPIOE
#define MALIBOTTI_RX_Pin GPIO_PIN_9
#define MALIBOTTI_RX_GPIO_Port GPIOE
#define MALIBOTTI_TX_Pin GPIO_PIN_10
#define MALIBOTTI_TX_GPIO_Port GPIOE
#define ETHERCAT_CS_Pin GPIO_PIN_11
#define ETHERCAT_CS_GPIO_Port GPIOE
#define EXTSPI_SCK_Pin GPIO_PIN_12
#define EXTSPI_SCK_GPIO_Port GPIOE
#define EXTSPI_MISO_Pin GPIO_PIN_13
#define EXTSPI_MISO_GPIO_Port GPIOE
#define EXTSPI_MOSI_Pin GPIO_PIN_14
#define EXTSPI_MOSI_GPIO_Port GPIOE
#define INFOLED_Pin GPIO_PIN_15
#define INFOLED_GPIO_Port GPIOE
#define DEBUG_TX_Pin GPIO_PIN_8
#define DEBUG_TX_GPIO_Port GPIOD
#define DEBUG_RX_Pin GPIO_PIN_9
#define DEBUG_RX_GPIO_Port GPIOD
#define VALVE1_Pin GPIO_PIN_10
#define VALVE1_GPIO_Port GPIOD
#define VALVE3_Pin GPIO_PIN_11
#define VALVE3_GPIO_Port GPIOD
#define CONVEYOR_Pin GPIO_PIN_14
#define CONVEYOR_GPIO_Port GPIOD
#define COMPRESSOR_Pin GPIO_PIN_15
#define COMPRESSOR_GPIO_Port GPIOD
#define LIGHTBARRIER1_Pin GPIO_PIN_6
#define LIGHTBARRIER1_GPIO_Port GPIOC
#define LIGHTBARRIER2_Pin GPIO_PIN_7
#define LIGHTBARRIER2_GPIO_Port GPIOC
#define LIGHTBARRIER3_Pin GPIO_PIN_8
#define LIGHTBARRIER3_GPIO_Port GPIOC
#define PA8_Pin GPIO_PIN_8
#define PA8_GPIO_Port GPIOA
#define STEPPER1_STEp_Pin GPIO_PIN_15
#define STEPPER1_STEp_GPIO_Port GPIOA
#define PD3_NOT_SPI_CLK_Pin GPIO_PIN_3
#define PD3_NOT_SPI_CLK_GPIO_Port GPIOD
#define CURRENT_ALERT_Pin GPIO_PIN_4
#define CURRENT_ALERT_GPIO_Port GPIOD
#define VALVE2_Pin GPIO_PIN_5
#define VALVE2_GPIO_Port GPIOD
#define STEPPER1_DIR_Pin GPIO_PIN_6
#define STEPPER1_DIR_GPIO_Port GPIOD
#define STEPPER2_DIR_Pin GPIO_PIN_7
#define STEPPER2_DIR_GPIO_Port GPIOD
#define STEPPER_RX_Pin GPIO_PIN_5
#define STEPPER_RX_GPIO_Port GPIOB
#define STEPPER_TX_Pin GPIO_PIN_6
#define STEPPER_TX_GPIO_Port GPIOB
#define STEPPER_EN_Pin GPIO_PIN_7
#define STEPPER_EN_GPIO_Port GPIOB
#define PE0_Pin GPIO_PIN_0
#define PE0_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
