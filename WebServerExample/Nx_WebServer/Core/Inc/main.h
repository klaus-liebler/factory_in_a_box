/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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
#define LED_GREEN_Pin GPIO_PIN_9
#define LED_GREEN_GPIO_Port GPIOI
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOF

/* USER CODE BEGIN Private defines */

/* Stage 1 Modbus I/O: pins verified against firmware_control_unit_ethercat (real target board) */
#define LIGHTBARRIER1_Pin GPIO_PIN_6
#define LIGHTBARRIER1_GPIO_Port GPIOC
#define LIGHTBARRIER2_Pin GPIO_PIN_7
#define LIGHTBARRIER2_GPIO_Port GPIOC
/* LIGHTBARRIER3 (PC8) intentionally not wired on this eval board: PC8 is used by SDMMC1_D0 (FileX website hosting) */

#define VALVE1_Pin GPIO_PIN_10
#define VALVE1_GPIO_Port GPIOD
#define VALVE2_Pin GPIO_PIN_5
#define VALVE2_GPIO_Port GPIOD
#define VALVE3_Pin GPIO_PIN_11
#define VALVE3_GPIO_Port GPIOD

#define PRESSURESENSOR_Pin GPIO_PIN_4
#define PRESSURESENSOR_GPIO_Port GPIOA

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif
