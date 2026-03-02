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
#include "tmc2209.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "USBPowerDelivery.h"
#include "gitconstants.hh"
#include "hx711.hh"
#include "sigmoid_acceleration_profile_100_1000_5.hh"
#include "sigmoid_stepper.hh"


namespace pinsDevBoard {
constexpr std::string_view BOARD_NAME = "WeAct STM32G431 CoreBoard";
constexpr gpio::Pin RESET = gpio::Pin::PG10;
constexpr gpio::Pin USB_VSENSE = gpio::Pin::PB02;

constexpr gpio::Pin HX711_DATA = gpio::Pin::PC10;
constexpr gpio::Pin HX711_CLK = gpio::Pin::PC11;

constexpr gpio::Pin HX711_DATA_ALT = gpio::Pin::PC11;
constexpr gpio::Pin HX711_CLK_ALT = gpio::Pin::PC12;

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
constexpr gpio::Pin RESET = gpio::Pin::PG10;
constexpr gpio::Pin USB_VSENSE = gpio::Pin::PF01;
constexpr gpio::Pin IO3 = gpio::Pin::PD02;

constexpr gpio::Pin I2C1_SCL = gpio::Pin::PC04;
constexpr gpio::Pin I2C1_SDA = gpio::Pin::PF00;

constexpr gpio::Pin I2C2_SCL = gpio::Pin::PC08;
constexpr gpio::Pin I2C2_SDA = gpio::Pin::PC09;

constexpr gpio::Pin I2C3_SCL = gpio::Pin::PA15;
constexpr gpio::Pin I2C3_SDA = gpio::Pin::PB07;

constexpr gpio::Pin HX711_DATA = gpio::Pin::PC10;
constexpr gpio::Pin HX711_CLK = gpio::Pin::PC11;

constexpr gpio::Pin HX711_DATA_ALT = gpio::Pin::PC11;
constexpr gpio::Pin HX711_CLK_ALT = gpio::Pin::PC12;

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
namespace pins = pinsDevBoard;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

// Motor controller instance
// RampStepper global instance with pins
//static tmc2209::TMC2209 *g_motor{nullptr};
static SigmoidStepper g_ramp_stepper(pins::STEPPER1_STEP, pins::STEPPER1_DIR,
                                     &profile_100_1000_5);
static HX711 g_hx711(GPIOC, GPIO_PIN_11, GPIOC, GPIO_PIN_10); // CLK, DATA

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
  g_hx711.set_gain(128, 128); // Set gain for both channels
  g_hx711.set_scale(-44.25, -10.98);
  g_hx711.tare_all(10);
  log_info("HX711 initialized");
  
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
  // g_ramp_stepper.handle_loop();
  if(current_tick - last_weight_measurement_time >= 1000) {
    last_weight_measurement_time = current_tick;
    long weightA = 0;
	  long weightB = 0;

    // Measure the weight for channel A
    weightA = g_hx711.get_weight(10, CHANNEL_A);
    weightB = g_hx711.get_weight(10, CHANNEL_B);
    log_info("Weight: %ld %ld", weightA, weightB);
  } 
  
  HAL_Delay(20);
}
