
#include "gpio.hh"
#include "log.h"
#include "main.h"

#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_spi.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string_view>
#include "hx711.hh"

extern SPI_HandleTypeDef hspi3;
HX711 g_hx711_instance(&hspi3, gpio::Pin::PC11, Channel_And_Gain::CH_A_GAIN_128);
uint32_t last_measurement_time = 0;

extern "C" void app_setup(void) {
  log_info("Starting HX711 test with raw measurements");
  
  // Wait for HX711 DATA pin (PC11) to go LOW - indicates chip is ready
  uint32_t timeout = HAL_GetTick() + 5000;  // 5 second timeout
  while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) != GPIO_PIN_RESET) {
    if (HAL_GetTick() > timeout) {
      log_error("Timeout waiting for HX711 DATA pin to go LOW");
      break;
    }
    log_debug("Waiting for HX711 DATA pin (PC11) to go LOW before setup...");
    HAL_Delay(500);
  }
  
  // Check final DATA pin state before setup
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);
  log_info("GPIO PC11 (HX711 DATA) is now: %s", pin_state == GPIO_PIN_RESET ? "LOW" : "HIGH");
  
  g_hx711_instance.setup();
  last_measurement_time = HAL_GetTick();
  log_info("HX711 initialized, starting continuous measurements...");
}

extern "C" void app_loop(void) {
  g_hx711_instance.loop();
  uint32_t now = HAL_GetTick();
  if (now - last_measurement_time >= 1000) {
    double weight = 0.0;
    bool quality =g_hx711_instance.getLastReadingA(&weight);
    log_info("Raw measurement (Channel A): %.2f %s", weight, quality ? "OK" : "N/A");
    last_measurement_time = now;
  }
  HAL_Delay(200);
}
