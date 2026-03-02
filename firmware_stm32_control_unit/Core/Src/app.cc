#include "PDController.h"
#include "PDProtocolAnalyzer.h"
#include "PDSink.h"
#include "gpio.hh"
#include "log.h"
#include "main.h"
#include "single_led.hh"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_spi.h"
#include "tmc2209.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "USBPowerDelivery.h"
#include "gitconstants.hh"
//#include "hx711.hh"
#include "sigmoid_acceleration_profile_100_1000_5.hh"
#include "sigmoid_stepper.hh"


namespace pinsDevBoard {
constexpr std::string_view BOARD_NAME = "WeAct STM32G431 CoreBoard";
constexpr gpio::Pin RESET = gpio::Pin::PG10;
constexpr gpio::Pin USB_VSENSE = gpio::Pin::PB02;

constexpr gpio::Pin HX711_DATA = gpio::Pin::PC10;
constexpr gpio::Pin HX711_CLK = gpio::Pin::PC11;

constexpr gpio::Pin BOOT_U2_TX = gpio::Pin::PA02;
constexpr gpio::Pin BOOT_U2_RX = gpio::Pin::PA03;

constexpr gpio::Pin U3_TX = gpio::Pin::PB10;
constexpr gpio::Pin U3_RX = gpio::Pin::PB11;

constexpr gpio::Pin STEPPER_EN = gpio::Pin::PA08;

constexpr gpio::Pin TMC2209_EN = gpio::Pin::PA08;
constexpr gpio::Pin STEPPER1_DIR = gpio::Pin::PC05;
// Conflict with vsense! constexpr gpio::Pin STEPPER2_DIR = gpio::Pin::PB02;
constexpr gpio::Pin STEPPER1_STEP = gpio::Pin::PB00;
constexpr gpio::Pin STEPPER2_STEP = gpio::Pin::PB01;
constexpr gpio::Pin LED_PIN = gpio::Pin::PC06;
constexpr bool LED_ON_LEVEL = true; // LED is active high
constexpr int VOLTAGE_FROM_USBC = 5000; // default voltage if powered from USB-C
}; // namespace pinsDevBoard

namespace pinsV1 {
  constexpr std::string_view BOARD_NAME = "FactoryInABox Controller Board v1.0";
constexpr gpio::Pin RESET = gpio::Pin::PG10;
constexpr gpio::Pin USB_VSENSE = gpio::Pin::PF01;
constexpr gpio::Pin IO3 = gpio::Pin::PD02;

constexpr gpio::Pin I2C1_SCL = gpio::Pin::PC04;
constexpr gpio::Pin I2C1_SDA = gpio::Pin::PF00;

constexpr gpio::Pin I2C2_SCL = gpio::Pin::PC08;
constexpr gpio::Pin I2C2_SDA = gpio::Pin::PC09;

constexpr gpio::Pin I2C3_SCL = gpio::Pin::PA15;
constexpr gpio::Pin I2C3_SDA = gpio::Pin::PB07;

constexpr gpio::Pin HX711_DATA = gpio::Pin::PC11;
constexpr gpio::Pin HX711_CLK = gpio::Pin::PC12;


constexpr gpio::Pin BOOT_U2_TX = gpio::Pin::PA02;
constexpr gpio::Pin BOOT_U2_RX = gpio::Pin::PA03;

constexpr gpio::Pin U3_TX = gpio::Pin::PB10;
constexpr gpio::Pin U3_RX = gpio::Pin::PB11;

constexpr gpio::Pin SPI1_NSS = gpio::Pin::PA04;
constexpr gpio::Pin SPI1_CLK = gpio::Pin::PA05;
constexpr gpio::Pin SPI1_MISO = gpio::Pin::PA06;
constexpr gpio::Pin SPI1_MOSI = gpio::Pin::PA07;

constexpr gpio::Pin STEPPER_EN = gpio::Pin::PA08;

constexpr gpio::Pin TMC2209_EN = gpio::Pin::PA08;
constexpr gpio::Pin STEPPER1_DIR = gpio::Pin::PC05;
constexpr gpio::Pin STEPPER2_DIR = gpio::Pin::PB02;
constexpr gpio::Pin STEPPER1_STEP = gpio::Pin::PB00;
constexpr gpio::Pin STEPPER2_STEP = gpio::Pin::PB01;
constexpr gpio::Pin LED_PIN = gpio::Pin::PB12;
constexpr bool LED_ON_LEVEL = false; // LED is active high
constexpr int VOLTAGE_FROM_USBC = 15000; // default voltage if powered from USB-C
}; // namespace pinsV1

// Namespace alias to allow easy switching between pin versions
namespace pins = pinsV1;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi3;

// Motor controller instance
// RampStepper global instance with pins
//static tmc2209::TMC2209 *g_motor{nullptr};
static SigmoidStepper g_ramp_stepper(pins::STEPPER1_STEP, pins::STEPPER1_DIR,
                                     &profile_100_1000_5);
//static HX711 g_hx711(GPIOC, GPIO_PIN_12, GPIOC, GPIO_PIN_11); // CLK, DATA

#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

#define CALIBRATION_X1 78070 // tare
#define CALIBRATION_Y1 0.0 // grams
#define CALIBRATION_X2 -81080 // loaded
#define CALIBRATION_Y2 222.22 // grams

#define SPI3_DMA_REPRO_TEST 0

uint8_t TxData[8] =
{ 0b0000000, 0x00000001, 0b01010101, 0b01010101, 0b01010101, 0b01010101,
		0b01010101, 0b01010101 };  // clock, 25 ticks
uint8_t RxData[8];
uint32_t raw_reading;
double converted_reading;
union
{
	uint32_t uint32_value;
	int32_t int32_value;
} uint2int_converter;

volatile bool sensor_read_flag = false;
volatile bool sensor_dma_in_progress = false;

#if SPI3_DMA_REPRO_TEST
volatile uint32_t spi3_dma_test_start_count = 0;
volatile uint32_t spi3_dma_test_cplt_count = 0;
volatile uint32_t spi3_dma_test_error_count = 0;
volatile uint32_t spi3_dma_test_last_error = 0;
static uint32_t spi3_dma_test_last_log_tick = 0;
#endif

double measured_weight;

static void spi3_prepare_for_dma_start() {
  // Fix for OVR corruption: Reset DMA channels via DeInit/Init
  // Root cause: After OVR, DMA channel state gets corrupted and requires CCR reset
  __HAL_SPI_DISABLE(&hspi3);
  
  // DeInit and reinit DMA channels (resets CCR and clears corrupted state)
  HAL_DMA_DeInit(hspi3.hdmarx);
  HAL_DMA_Init(hspi3.hdmarx);
  HAL_DMA_DeInit(hspi3.hdmatx);
  HAL_DMA_Init(hspi3.hdmatx);
  
  // Re-link DMA handles to SPI (DeInit cleared these)
  __HAL_LINKDMA(&hspi3, hdmarx, *hspi3.hdmarx);
  __HAL_LINKDMA(&hspi3, hdmatx, *hspi3.hdmatx);
  
  // Drain RX FIFO
  while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_RXNE)) {
    volatile uint8_t dummy __attribute__((unused)) = *(__IO uint8_t *)&hspi3.Instance->DR;
  }
  
  // Clear OVR flag
  if (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_OVR)) {
    __HAL_SPI_CLEAR_OVRFLAG(&hspi3);
  }
  
  hspi3.ErrorCode = HAL_SPI_ERROR_NONE;
  
  __HAL_SPI_ENABLE(&hspi3);
}

// Recovery function removed - spi3_prepare_for_dma_start() now handles DMA reset properly

#if SPI3_DMA_REPRO_TEST
static void spi3_dma_repro_start() {
  if (sensor_dma_in_progress) {
    return;
  }

  // Test: Use HAL_DMA_DeInit/Init instead of full SPI DeInit/Init
  __HAL_SPI_DISABLE(&hspi3);
  
  // Save DMA handles before DeInit
  DMA_HandleTypeDef* hdmarx = hspi3.hdmarx;
  DMA_HandleTypeDef* hdmatx = hspi3.hdmatx;
  
  // DeInit and reinit DMA channels (resets CCR and clears corrupted state)
  HAL_DMA_DeInit(hdmarx);
  HAL_DMA_Init(hdmarx);
  HAL_DMA_DeInit(hdmatx);
  HAL_DMA_Init(hdmatx);
  
  // Re-link DMA handles to SPI (DeInit cleared these)
  __HAL_LINKDMA(&hspi3, hdmarx, *hdmarx);
  __HAL_LINKDMA(&hspi3, hdmatx, *hdmatx);
  
  // Drain RX FIFO
  while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_RXNE)) {
    volatile uint8_t dummy __attribute__((unused)) = *(__IO uint8_t *)&hspi3.Instance->DR;
  }
  
  // Clear OVR flag
  if (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_OVR)) {
    __HAL_SPI_CLEAR_OVRFLAG(&hspi3);
  }
  
  __HAL_SPI_ENABLE(&hspi3);

  if (HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) (&TxData),
                                  (uint8_t*) (&RxData), sizeof(TxData)) == HAL_OK) {
    sensor_dma_in_progress = true;
    spi3_dma_test_start_count = spi3_dma_test_start_count + 1U;
  }
}

static void spi3_dma_repro_loop(uint32_t current_tick) {
  if (!sensor_dma_in_progress) {
    spi3_dma_repro_start();
  }

  if (current_tick - spi3_dma_test_last_log_tick >= 1000U) {
    spi3_dma_test_last_log_tick = current_tick;
    log_info("SPI3 DMA REPRO: start=%lu cplt=%lu err=%lu lastErr=0x%08lX state=%lu",
             (unsigned long)spi3_dma_test_start_count,
             (unsigned long)spi3_dma_test_cplt_count,
             (unsigned long)spi3_dma_test_error_count,
             (unsigned long)spi3_dma_test_last_error,
             (unsigned long)hspi3.State);
  }
}
#endif

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI3)
	{
    #if SPI3_DMA_REPRO_TEST
    spi3_dma_test_cplt_count = spi3_dma_test_cplt_count + 1U;
    #endif
		sensor_read_flag = true;
    sensor_dma_in_progress = false;
	}
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI3)
  {
    uint32_t error_code = hspi->ErrorCode;
    sensor_dma_in_progress = false;
    sensor_read_flag = false;

#if SPI3_DMA_REPRO_TEST
    spi3_dma_test_error_count = spi3_dma_test_error_count + 1U;
    spi3_dma_test_last_error = error_code;
#endif

    (void)HAL_SPI_DMAStop(hspi);

    if ((error_code & HAL_SPI_ERROR_OVR) != 0U) {
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
      hspi->ErrorCode = HAL_SPI_ERROR_NONE;
      log_warn("SPI3 OVR detected and cleared");
    }

    log_error("SPI3 DMA error: ErrorCode=0x%08lX State=%lu", (unsigned long)error_code,
              (unsigned long)hspi->State);
  }
}


void convert_weight(){
  			raw_reading = (((uint32_t) (RxData[7] & 0b00000010)) >> 1)
					| (((uint32_t) (RxData[7] & 0b00001000)) >> 2)
					| (((uint32_t) (RxData[7] & 0b00100000)) >> 3)
					| (((uint32_t) (RxData[7] & 0b10000000)) >> 4)
					| (((uint32_t) (RxData[6] & 0b00000010)) << 3)
					| (((uint32_t) (RxData[6] & 0b00001000)) << 2)
					| (((uint32_t) (RxData[6] & 0b00100000)) << 1)
					| (((uint32_t) (RxData[6] & 0b10000000)) << 0) |

					(((uint32_t) (RxData[5] & 0b00000010)) << 7)
					| (((uint32_t) (RxData[5] & 0b00001000)) << 6)
					| (((uint32_t) (RxData[5] & 0b00100000)) << 5)
					| (((uint32_t) (RxData[5] & 0b10000000)) << 4)
					| (((uint32_t) (RxData[4] & 0b00000010)) << 11)
					| (((uint32_t) (RxData[4] & 0b00001000)) << 10)
					| (((uint32_t) (RxData[4] & 0b00100000)) << 9)
					| (((uint32_t) (RxData[4] & 0b10000000)) << 8) |

					(((uint32_t) (RxData[3] & 0b00000010)) << 15)
					| (((uint32_t) (RxData[3] & 0b00001000)) << 14)
					| (((uint32_t) (RxData[3] & 0b00100000)) << 13)
					| (((uint32_t) (RxData[3] & 0b10000000)) << 12)
					| (((uint32_t) (RxData[2] & 0b00000010)) << 19)
					| (((uint32_t) (RxData[2] & 0b00001000)) << 18)
					| (((uint32_t) (RxData[2] & 0b00100000)) << 17)
					| (((uint32_t) (RxData[2] & 0b10000000)) << 16) |

					(((uint32_t) (RxData[1] & 0b00000010)) << 23)
					| (((uint32_t) (RxData[1] & 0b00001000)) << 22)
					| (((uint32_t) (RxData[1] & 0b00100000)) << 21)
					| (((uint32_t) (RxData[1] & 0b10000000)) << 20);

			raw_reading = raw_reading << 8; // 24-bit two's complement in 32-bit variable
			uint2int_converter.uint32_value = raw_reading;
			converted_reading = ((double) uint2int_converter.int32_value)/ 256.0;

			measured_weight =
					((double) (CALIBRATION_Y2 - CALIBRATION_Y1))
							/ ((double) (CALIBRATION_X2 - CALIBRATION_X1))
							* (converted_reading - (double) CALIBRATION_X2)+ CALIBRATION_Y2;

			log_info("%.0f g    ", round(measured_weight));
}

static uint32_t last_tick = 0;

/**
 * UART RX complete callback - called by HAL
 * This needs to be in the main.c or called from there
 */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
    // Process the received byte
    // uint8_t rx_byte = cmd_buffer[0];
    // process_uart_data(rx_byte);

    // Re-enable receive for next byte
    // HAL_UART_Receive_IT(&huart2, cmd_buffer, 1);
  }
}

extern "C" void TIM1_TRG_COM_TIM17_IRQHandler(void) {
  if (TIM17->SR & TIM_SR_UIF) {
    TIM17->SR &= ~TIM_SR_UIF;
    // g_ramp_stepper.handle_update_interrupt_();
  }
}

single_led::M<pins::LED_ON_LEVEL> led(pins::LED_PIN);
single_led::BlinkPattern blink_pattern(200, 800);

void requestVoltage() {
  // check if desired voltage is supported
  for (int i = 0; i < PowerSink.numSourceCapabilities; i ++) {
    if (PowerSink.sourceCapabilities[i].minVoltage <= pins::VOLTAGE_FROM_USBC &&
        PowerSink.sourceCapabilities[i].maxVoltage >= pins::VOLTAGE_FROM_USBC) {
      PowerSink.requestPower(pins::VOLTAGE_FROM_USBC);
      log_info("Requested voltage %d mV from USB PD supply", pins::VOLTAGE_FROM_USBC);
      return;
    }
  }

  log_warn("Desired voltage %d mV is not supported", pins::VOLTAGE_FROM_USBC);
}

extern "C" void handleEvent(PDSinkEventType eventType) {

  log_debug("handleUSBCEvent. eventType=%d", eventType);
  if (eventType == PDSinkEventType::sourceCapabilitiesChanged) {
    // source capabilities have changed
    if (PowerSink.isConnected()) {
      log_info("USB PD supply is connected. Requesting suitable voltage...");
      requestVoltage();

    } else {
      log_info("No supply or no USB PD capable supply is connected");

      PowerSink.requestPower(5000); // reset to 5V
    }

  } else if (eventType == PDSinkEventType::voltageChanged) {
    // voltage has changed
    if (PowerSink.activeVoltage != 0) {
      log_info("Voltage: %d mV @ %d mA (max)", PowerSink.activeVoltage,
               PowerSink.activeCurrent);
    } else {
      log_info("Disconnected");
    }

  } else if (eventType == PDSinkEventType::powerRejected) {
    // rare case: power supply rejected requested power
    log_warn("Power request rejected");
    log_warn("Voltage: %d mV @ %d mA (max)", PowerSink.activeVoltage,
             PowerSink.activeCurrent);
  }
}

void greeting() {
  printf("\r\n\r\n\r\n");
  printf("=== FacoryInABox Controller Application Starting===\r\n");
  printf("Running on Board %.*s\r\n", (int)pins::BOARD_NAME.length(),
         pins::BOARD_NAME.data());
  // Print version information
  printf("=== Build Information ===\r\n");
  printf("  Version: %.*s\r\n", (int)git::VERSION.length(),
         git::VERSION.data());
  printf("  Commit: %.*s (%.*s)\r\n", (int)git::COMMIT_HASH.length(),
         git::COMMIT_HASH.data(), (int)git::BRANCH.length(),
         git::BRANCH.data());
  printf("  Message: %.*s\r\n", (int)git::COMMIT_MESSAGE.length(),
         git::COMMIT_MESSAGE.data());
  printf("  Author: %.*s\r\n", (int)git::COMMIT_AUTHOR.length(),
         git::COMMIT_AUTHOR.data());
  printf("  Built: %.*s\r\n", (int)git::BUILD_TIMESTAMP.length(),
         git::BUILD_TIMESTAMP.data());
  if (git::IS_DIRTY) {
    printf("⚠ Working directory has uncommitted changes!\r\n");
  } else {
    printf("Working directory was clean related to Git.\r\n");
  }
  printf("========================\r\n");
}

extern "C" void app_setup(void) {

  // Initialize GPIO
  gpio::Gpio::ConfigureGPIOOutput(pins::LED_PIN, false);
  gpio::Gpio::ConfigureGPIOOutput(pins::TMC2209_EN,
                                  true); // Enable TMC2209 driver
  gpio::Gpio::ConfigureGPIOOutput(pins::STEPPER1_DIR,
                                  false); // Set initial direction for motor
  gpio::Gpio::ConfigureGPIOOutput(pins::STEPPER1_STEP,
                                  false); // Initialize step pin low
  gpio::Gpio::ConfigureGPIOOutput(pins::STEPPER2_STEP,
                                  false); // Initialize step pin low

  //init hx711
  //g_hx711.set_gain(128, 128); // Set gain for both channels
  //g_hx711.set_scale(-44.25, -10.98);
  //g_hx711.tare_all(10);
  //log_info("HX711 initialized");

  #if SPI3_DMA_REPRO_TEST
  sensor_dma_in_progress = false;
  spi3_dma_repro_start();
  log_info("SPI3 DMA repro mode active");
  #else
  spi3_prepare_for_dma_start();
  if (HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) (&TxData),
			(uint8_t*) (&RxData), sizeof(TxData)) == HAL_OK) {
    sensor_dma_in_progress = true;
    log_info("HX711 SPI DMA started");
  } else {
    log_warn("Initial HX711 DMA start failed");
  }
  #endif
  
                                  // Init USB PD sink
  PowerSink.start(handleEvent);

  // Initialize ramp stepper subsystem
  // g_ramp_stepper.init();

  /*
  // Create motor controller instance
  g_motor = new tmc2209::TMC2209(&huart3, 0, pins::TMC2209_EN);

  // Initialize motor
  if (g_motor->initForNormalSpeedAndUartBasedOperation(false)) {
    log_info("Motor initialized successfully");
  } else {
    log_error("Failed to initialize motor");
  }
  g_motor->printPrettyFullSystemState();
  g_motor->performStealthChopAutoTuningForQuietOperation();
*/
  led.AnimatePixel(HAL_GetTick(), &blink_pattern);

  last_tick = HAL_GetTick();
}
int64_t last_weight_measurement_time = 0;
// Loop-Funktion: im Hauptloop aufgerufen
extern "C" void app_loop(void) {
  uint32_t current_tick = HAL_GetTick();
  led.Loop(current_tick);
  // PDProtocolAnalyzer.poll();
  PowerSink.poll();

  #if SPI3_DMA_REPRO_TEST
  spi3_dma_repro_loop(current_tick);
  HAL_Delay(20);
  return;
  #endif

  // g_ramp_stepper.handle_loop();
  if(current_tick - last_weight_measurement_time >= 1000 && sensor_read_flag) {
    last_weight_measurement_time = current_tick;
    //long weightA = 0;
	  //long weightB = 0;

    // Measure the weight for channel A
    //weightA = g_hx711.get_weight(10, CHANNEL_A);
    //weightB = g_hx711.get_weight(10, CHANNEL_B);
    //log_info("Weight: %ld %ld", weightA, weightB);
    convert_weight();
    sensor_read_flag = false;
  } 

  if ((sensor_dma_in_progress == false)
				&& (hspi3.State == HAL_SPI_STATE_READY)
				&& (HAL_GPIO_ReadPin(HX711_DATA_GPIO_Port, HX711_DATA_Pin)
						== GPIO_PIN_RESET))
		{
      spi3_prepare_for_dma_start();
      if (HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) (&TxData),
          (uint8_t*) (&RxData), sizeof(TxData)) == HAL_OK) {
        sensor_dma_in_progress = true;
      } else {
        log_warn("HX711 DMA restart failed: state=%lu err=0x%08lX",
                 (unsigned long)hspi3.State, (unsigned long)hspi3.ErrorCode);
      }
		}
  
  HAL_Delay(20);
}
