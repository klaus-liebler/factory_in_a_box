/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h5xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h5xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// CubeMX generiert unten leere SVC_Handler/PendSV_Handler/SysTick_Handler-Bodies,
// solange ThreadX nicht als CubeMX-Middleware-Komponente eingetragen ist (bewusst so,
// s. CMakeLists.txt/libs/-Kommentar). Diese drei Symbole kollidieren beim Linken mit
// ThreadX' eigenen starken Definitionen derselben Namen (siehe
// libs/ST/threadx/ports/cortex_m33/gnu/src/tx_thread_schedule.S sowie
// Core/Src/tx_initialize_low_level.S). Umbenennung per Makro statt manueller Loeschung
// nach jedem "Generate Code": die leeren Bodies unten landen unter einem harmlosen
// Alias-Namen (vom Linker als toter Code entfernt), der echte Symbolname faellt ueber
// den weak-Default der Startup-Datei auf ThreadX' Implementierung durch.
#define SVC_Handler SVC_Handler_MX_unused
#define PendSV_Handler PendSV_Handler_MX_unused
#define SysTick_Handler SysTick_Handler_MX_unused
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef huart3;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern SD_HandleTypeDef hsd1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel0;
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef handle_GPDMA1_Channel3;
extern DMA_HandleTypeDef handle_GPDMA1_Channel2;
extern UART_HandleTypeDef huart5;
/* USER CODE BEGIN EV */


/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/* USER CODE BEGIN HardFault_Handler_C */
// Rohes, HAL-freies Polling-Transmit fuer den Fault-Handler: HAL_UART_Transmit() (wie es sonst
// ueberall in diesem Projekt genutzt wird, s. syscalls.c::_write()) haengt __HAL_LOCK()/
// __HAL_UNLOCK() um sich herum -- traf der Fault WAEHREND eine vorherige HAL_UART_Transmit()
// noch lief (huart3->Lock also noch HAL_LOCKED, weil deren regulaerer Rueckweg nie erreicht
// wurde), liefert ein erneuter HAL_UART_Transmit()-Aufruf sofort HAL_BUSY zurueck OHNE ein
// einziges Byte zu senden -- exakt das beobachtete "Handler gibt nichts aus". Direkter
// Registerzugriff auf USART3 ignoriert diesen Software-Lock komplett (Standardmuster fuer
// Fault-Handler: der Hardware-Zustand ist das einzig Verlaessliche, sobald die Ausnahme
// eingetreten ist).
static void hardfault_uart_puts(char const *s) {
    while (*s) {
        while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0) {
        }
        USART3->TDR = (uint8_t)*s;
        s++;
    }
    while ((USART3->ISR & USART_ISR_TC) == 0) {
    }
}

// Diagnose-Erweiterung (Klaus Liebler, USBX-Bringup): der urspruengliche Handler gab nur eine
// statische Meldung aus, ohne Faultort/-ursache -- bei einem HardFault waehrend eines
// mehrpaketigen EP0-IN-Transfers (grosser GET_DESCRIPTOR(CONFIGURATION)-Response, 211 Byte > ein
// 64-Byte-Paket) ist das ohne angehaengten Debugger nutzlos. HardFault_Handler unten ist deshalb
// ein "naked" Trampolin (kein Prolog/Epilog, damit LR beim Eintritt noch den EXC_RETURN-Wert
// traegt), das anhand Bit 2 von EXC_RETURN erkennt, ob MSP oder PSP den Stackframe zum Zeitpunkt
// des Faults haelt, und diesen Stackpointer an HardFault_Handler_C uebergibt -- Standardmuster
// fuer Cortex-M-HardFault-Diagnose ohne Debugger, unveraendert gueltig auf M33 (CFSR/HFSR/MMFAR/
// BFAR liegen an denselben SCB-RegisterOffsets).
void HardFault_Handler_C(uint32_t *stacked_regs)
{
    // Stackframe-Layout, das die CPU beim Exception-Entry automatisch ablegt (Cortex-M
    // Architecture Reference Manual, B1.5.6): r0,r1,r2,r3,r12,lr,pc,xpsr.
    uint32_t stacked_r0 = stacked_regs[0];
    uint32_t stacked_r1 = stacked_regs[1];
    uint32_t stacked_r2 = stacked_regs[2];
    uint32_t stacked_r3 = stacked_regs[3];
    uint32_t stacked_r12 = stacked_regs[4];
    uint32_t stacked_lr = stacked_regs[5];
    uint32_t stacked_pc = stacked_regs[6];
    uint32_t stacked_psr = stacked_regs[7];
    uint32_t cfsr = SCB->CFSR;
    uint32_t hfsr = SCB->HFSR;
    uint32_t mmfar = SCB->MMFAR;
    uint32_t bfar = SCB->BFAR;

    char buf[320];
    int len = snprintf(buf, sizeof(buf),
        "\r\n!!! HardFault_Handler !!!\r\n"
        "PC=0x%08lX LR=0x%08lX PSR=0x%08lX SP=0x%08lX\r\n"
        "R0=0x%08lX R1=0x%08lX R2=0x%08lX R3=0x%08lX R12=0x%08lX\r\n"
        "CFSR=0x%08lX (MMFSR=0x%02lX BFSR=0x%02lX UFSR=0x%04lX) HFSR=0x%08lX MMFAR=0x%08lX BFAR=0x%08lX\r\n",
        (unsigned long)stacked_pc, (unsigned long)stacked_lr, (unsigned long)stacked_psr, (unsigned long)(uintptr_t)stacked_regs,
        (unsigned long)stacked_r0, (unsigned long)stacked_r1, (unsigned long)stacked_r2, (unsigned long)stacked_r3, (unsigned long)stacked_r12,
        (unsigned long)cfsr, (unsigned long)(cfsr & 0xFFu), (unsigned long)((cfsr >> 8) & 0xFFu), (unsigned long)((cfsr >> 16) & 0xFFFFu),
        (unsigned long)hfsr, (unsigned long)mmfar, (unsigned long)bfar);
    if (len > 0) {
        hardfault_uart_puts(buf);
    }
    while (1)
    {
    }
}
/* USER CODE END HardFault_Handler_C */

/**
  * @brief This function handles Hard fault interrupt.
  */
__attribute__((naked)) void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  __asm volatile(
    "movs r0, #4        \n"
    "mov  r1, lr        \n"
    "tst  r0, r1        \n"
    "beq  1f            \n"
    "mrs  r0, psp       \n"
    "b    2f            \n"
    "1:                 \n"
    "mrs  r0, msp       \n"
    "2:                 \n"
    "b    HardFault_Handler_C \n"
  );
  /* USER CODE END HardFault_IRQn 0 */
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H5xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h5xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles GPDMA1 Channel 0 global interrupt.
  */
void GPDMA1_Channel0_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel0_IRQn 0 */

  /* USER CODE END GPDMA1_Channel0_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel0);
  /* USER CODE BEGIN GPDMA1_Channel0_IRQn 1 */

  /* USER CODE END GPDMA1_Channel0_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 1 global interrupt.
  */
void GPDMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel1_IRQn 0 */

  /* USER CODE END GPDMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel1);
  /* USER CODE BEGIN GPDMA1_Channel1_IRQn 1 */

  /* USER CODE END GPDMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 2 global interrupt.
  */
void GPDMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel2_IRQn 0 */

  /* USER CODE END GPDMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel2);
  /* USER CODE BEGIN GPDMA1_Channel2_IRQn 1 */

  /* USER CODE END GPDMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 3 global interrupt.
  */
void GPDMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel3_IRQn 0 */

  /* USER CODE END GPDMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel3);
  /* USER CODE BEGIN GPDMA1_Channel3_IRQn 1 */

  /* USER CODE END GPDMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles SDMMC1 global interrupt.
  */
void SDMMC1_IRQHandler(void)
{
  /* USER CODE BEGIN SDMMC1_IRQn 0 */

  /* USER CODE END SDMMC1_IRQn 0 */
  HAL_SD_IRQHandler(&hsd1);
  /* USER CODE BEGIN SDMMC1_IRQn 1 */

  /* USER CODE END SDMMC1_IRQn 1 */
}

/**
  * @brief This function handles Ethernet global interrupt.
  */
void ETH_IRQHandler(void)
{
  /* USER CODE BEGIN ETH_IRQn 0 */

  /* USER CODE END ETH_IRQn 0 */
  HAL_ETH_IRQHandler(&heth);
  /* USER CODE BEGIN ETH_IRQn 1 */

  /* USER CODE END ETH_IRQn 1 */
}

/**
  * @brief This function handles Ethernet Wakeup global interrupt.
  */
void ETH_WKUP_IRQHandler(void)
{
  /* USER CODE BEGIN ETH_WKUP_IRQn 0 */

  /* USER CODE END ETH_WKUP_IRQn 0 */
  HAL_ETH_IRQHandler(&heth);
  /* USER CODE BEGIN ETH_WKUP_IRQn 1 */

  /* USER CODE END ETH_WKUP_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
