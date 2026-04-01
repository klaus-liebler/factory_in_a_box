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
constexpr int VOLTAGE_FROM_USBC = 20000; // default voltage if powered from USB-C
}; // namespace pinsV1

// Namespace alias to allow easy switching between pin versions
namespace pins = pinsV1;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi3;

// Motor controller instance
// RampStepper global instance with pins
tmc2209::TMC2209 g_motor(&huart3, 0, pins::TMC2209_EN);
SigmoidStepper g_ramp_stepper(pins::STEPPER1_STEP, pins::STEPPER1_DIR,
                                     &profile_100_1000_5);

namespace {
constexpr uint8_t kIhold = 2;
constexpr uint8_t kIrun = 5;
constexpr uint8_t kIholdDelay = 4;
} // namespace


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
    g_ramp_stepper.Handle_update_interrupt_();
  }
}

single_led::M<pins::LED_ON_LEVEL> led(pins::LED_PIN);
single_led::BlinkPattern blink_pattern(200, 800);

extern "C" void handleEvent(PDSinkEventType eventType) {
  switch (eventType) {
    case PDSinkEventType::sourceCapabilitiesChanged:
       if (PowerSink.isConnected()) {
        PowerSink.TryRequestPreferredVoltages({pins::VOLTAGE_FROM_USBC, 5000});
      }
      break;
    case PDSinkEventType::voltageChanged:
      log_info("Voltage: %d mV @ %d mA (max)", PowerSink.activeVoltage, PowerSink.activeCurrent);
      break;
    default:
      log_info("USB-PD Event: %d", static_cast<int>(eventType));
      break;
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
                                  true); // Disable TMC2209 driver
  HAL_Delay(50); // Short delay to ensure driver is disabled before changing EN pin state
  gpio::Gpio::ConfigureGPIOOutput(pins::STEPPER1_DIR,
                                  false); // Set initial direction for motor
    gpio::Gpio::ConfigureGPIOOutput(pins::STEPPER2_DIR,
                                  false); // Set initial direction for motor
  gpio::Gpio::ConfigureGPIOOutput(pins::STEPPER1_STEP,
                                  false); // Initialize step pin low
  gpio::Gpio::ConfigureGPIOOutput(pins::STEPPER2_STEP,
                                  false); // Initialize step pin low

  g_ramp_stepper.Init();
  // Init USB PD sink
  PowerSink.start(handleEvent);
  led.AnimatePixel(HAL_GetTick(), &blink_pattern);
  while(PowerSink.activeVoltage < 15000) {
    uint32_t current_tick = HAL_GetTick();
    PowerSink.Loop();
    led.Loop(current_tick);
    HAL_Delay(1);
  }
    
  log_info("USB-PD target voltage reached.");
  if (!g_motor.InitForNormalSpeedAndUartBasedOperation()) {
    log_error("Failed to initialize motor after USB-PD voltage became available");
  }
  g_motor.PerformStealthChopAutoTuningForQuietOperation();
  if (!g_motor.setIHoldIRun(kIhold, kIrun, kIholdDelay)) {
    log_error("Failed to set IHOLD_IRUN for low-load stealthChop profile");
  }
  log_info("Motor initialized successfully");
  g_motor.PrintPrettyFullSystemState();
  log_info("Starting motor with no-load quiet ramp");
  // Keep no-load operation away from strong audible resonance bands.
  constexpr int kRampStepsPerSecond[] = {120, 220, 320, 400};
  constexpr uint32_t kRampStepDelayMs = 60;
  for (int sps : kRampStepsPerSecond) {
    if (!g_motor.GenerateSteps(sps)) {
      log_error("Failed to start motor after initialization");
      break;
    }
    HAL_Delay(kRampStepDelayMs);
  }
}

uint32_t lastDebugOutput=0;
// Loop-Funktion: im Hauptloop aufgerufen
extern "C" void app_loop(void) {
  uint32_t current_tick = HAL_GetTick();
  led.Loop(current_tick);
  PowerSink.Loop();
  if(current_tick - lastDebugOutput > 1000){
    log_info("app_loop is running. Current Stepper Position: %ld. Tick: %lu", g_ramp_stepper.GetCurrentPosition(), current_tick);
    lastDebugOutput = current_tick;
  }
  HAL_Delay(10);
}
