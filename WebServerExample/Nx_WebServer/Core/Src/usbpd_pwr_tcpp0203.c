/**
 * @file usbpd_pwr_tcpp0203.c
 * @brief Minimal bring-up of the TCPP03-M20 Type-C port protection IC sitting
 *        between UCPD1 and the USB-C receptacle on the STM32H573I-DK.
 *
 * The TCPP03 boots into HIBERNATE (dead-battery) mode, in which it does not
 * present CC1/CC2 correctly for sink operation. It has to be woken up and
 * switched to at least LOWPOWER mode over I2C4 before UCPD1 can see anything
 * on CC1/CC2 -- confirmed against ST's own BSP
 * (Drivers/BSP/STM32H573I-DK/stm32h573i_discovery_usbpd_pwr.c in
 * STM32Cube_FW_H5). This file only implements what's needed for that: I2C4
 * bring-up, the ENABLE GPIO, and the Init/SetPowerMode(LOWPOWER) sequence.
 * VBUS voltage/current sensing (ADC2) and the FLGn fault interrupt (PG1/EXTI1)
 * from the full BSP are intentionally not ported -- not needed just to read
 * source capabilities as a sink.
 */

#include "stm32h5xx_hal.h"
#include "tcpp0203.h"

/* PCLK1 = 250 MHz in this project's SystemClock_Config() (see main.c: PLLM=5,
 * PLLN=100, PLLP=2, APB1 prescaler = 1). TIMING value computed with ST's own
 * I2C_GetTiming() algorithm (Drivers/BSP/STM32H573I-DK/stm32h573i_discovery_bus.c)
 * for 100kHz Standard Mode at that clock -- not a guessed/generic value. */
#define TCPP0203_I2C_TIMING        0xF0D04648U

#define TCPP0203_ENABLE_GPIO_PORT  GPIOG
#define TCPP0203_ENABLE_GPIO_PIN   GPIO_PIN_0

static I2C_HandleTypeDef hi2c4;
static TCPP0203_Object_t tcpp0203_obj;

/* HAL_Delay() spins on HAL_GetTick(), which only advances via the TIM6 tick
 * interrupt. TCPP0203_WakeupForSink() runs from tx_application_define(), which
 * ThreadX calls with all interrupts globally masked (CPSID i in
 * tx_initialize_low_level.S, only lifted once the scheduler starts) -- so
 * HAL_Delay() would wait forever for a tick that can never fire. Busy-wait on
 * the DWT cycle counter instead, which free-runs off the core clock and needs
 * no interrupt (same reasoning as the micros() fix in stm32_libs/common_stm32).
 */
static void cpu_delay_ms(uint32_t ms)
{
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000U) * ms;
    while ((DWT->CYCCNT - start) < cycles)
    {
        /* busy wait */
    }
}

static int32_t tcpp0203_io_init(void)
{
    return 0;
}

static int32_t tcpp0203_io_deinit(void)
{
    return 0;
}

static int32_t tcpp0203_io_get_tick(void)
{
    return (int32_t)HAL_GetTick();
}

static int32_t tcpp0203_io_write_reg(uint16_t dev_addr, uint16_t reg, uint8_t *data, uint16_t length)
{
    return (HAL_I2C_Mem_Write(&hi2c4, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, data, length, 100) == HAL_OK) ? 0 : -1;
}

static int32_t tcpp0203_io_read_reg(uint16_t dev_addr, uint16_t reg, uint8_t *data, uint16_t length)
{
    return (HAL_I2C_Mem_Read(&hi2c4, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, data, length, 100) == HAL_OK) ? 0 : -1;
}

static void tcpp0203_gpio_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    /* I2C4: SCL=PB8, SDA=PB9, AF6, open-drain with pull-up (matches the
       STM32H573I-DK BSP's I2C4_MspInit()). */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio_init.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio_init.Mode = GPIO_MODE_AF_OD;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_init.Alternate = GPIO_AF6_I2C4;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    /* TCPP03 ENABLE pin (PG0, push-pull output). Held low here; driven high
       in TCPP0203_WakeupForSink() once I2C is ready. */
    __HAL_RCC_GPIOG_CLK_ENABLE();
    gpio_init.Pin = TCPP0203_ENABLE_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TCPP0203_ENABLE_GPIO_PORT, &gpio_init);
    HAL_GPIO_WritePin(TCPP0203_ENABLE_GPIO_PORT, TCPP0203_ENABLE_GPIO_PIN, GPIO_PIN_RESET);
}

int TCPP0203_WakeupForSink(void)
{
    tcpp0203_gpio_init();

    __HAL_RCC_I2C4_CLK_ENABLE();
    hi2c4.Instance = I2C4;
    hi2c4.Init.Timing = TCPP0203_I2C_TIMING;
    hi2c4.Init.OwnAddress1 = 0;
    hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c4.Init.OwnAddress2 = 0;
    hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c4) != HAL_OK)
    {
        return -1;
    }

    HAL_GPIO_WritePin(TCPP0203_ENABLE_GPIO_PORT, TCPP0203_ENABLE_GPIO_PIN, GPIO_PIN_SET);
    cpu_delay_ms(10);

    TCPP0203_IO_t io_ctx = {0};
    io_ctx.Init = tcpp0203_io_init;
    io_ctx.DeInit = tcpp0203_io_deinit;
    io_ctx.Address = TCPP0203_I2C_ADDRESS_X68;
    io_ctx.WriteReg = tcpp0203_io_write_reg;
    io_ctx.ReadReg = tcpp0203_io_read_reg;
    io_ctx.GetTick = tcpp0203_io_get_tick;

    if (TCPP0203_RegisterBusIO(&tcpp0203_obj, &io_ctx) != TCPP0203_OK)
    {
        return -1;
    }
    if (TCPP0203_Init(&tcpp0203_obj) != TCPP0203_OK)
    {
        return -1;
    }
    /* HIBERNATE (power-up default) -> LOWPOWER: this is what actually makes the
       TCPP03 present CC1/CC2 for sink detection (see ST BSP_USBPD_PWR_SetPowerMode()
       comment: "Mode is set to Low power to enable TCPP03 behavior on CC lines"). */
    if (TCPP0203_SetPowerMode(&tcpp0203_obj, TCPP0203_POWER_MODE_LOWPOWER) != TCPP0203_OK)
    {
        return -1;
    }

    return 0;
}
