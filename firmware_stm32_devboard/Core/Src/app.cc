
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
HX711 g_hx711_instance(&hspi3, gpio::Pin::PC11, {Channel_And_Gain::CH_A_GAIN_64});
//HX711 g_hx711_instance(gpio::Pin::PB05, gpio::Pin::PC11, {Channel_And_Gain::CH_A_GAIN_128, Channel_And_Gain::CH_B_GAIN_32});
uint32_t last_measurement_time = 0;

extern "C" void app_setup(void) {
  
  log_info("Starting HX711 test with raw measurements");
  g_hx711_instance.setup();
  last_measurement_time = HAL_GetTick();
  log_info("HX711 initialized, starting continuous measurements...");
}

extern "C" void app_loop(void) {
  g_hx711_instance.loop();
  uint32_t now = HAL_GetTick();
  if (now - last_measurement_time >= 1000) {
    int32_t weight = 0;
    bool quality =g_hx711_instance.getLastRawValueA(&weight);
    log_info("Raw measurement (Channel A): %d %s", weight, quality ? "OK" : "N/A");
    last_measurement_time = now;
  }
  HAL_Delay(20);
}
