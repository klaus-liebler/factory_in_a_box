#include "log.h"
#include "main.h"
#include <cmath>
#include <cstring>
#include <cstdio>
#include "TMC2209.hpp"
#include "gpio.hh"
#include "single_led.hh"
#include "log.h"
#include "stm32g4xx_hal.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

// Private variables
static uint32_t last_tick = 0;
static int loop_count = 0;
const uint32_t LOOP_INTERVAL_MS = 1000;

// Test-Funktion: alle TMC2209 an huart3 abfragen
static void test_tmc2209_devices(void) {
    log_info("TMC2209: Starting device test on huart3");

    
    for (uint8_t id = 0; id < 4; id++) {
        // Erstelle einen Treiber mit EN-Pin PA8
        tmc2209::TMC2209 test_driver(&huart3, id, gpio::Pin::PA08);
        test_driver.enable();
        
        // Versuche GSTAT zu lesen (Register 0x01)
        uint32_t gstat = 0;
        bool found = test_driver.readRegister(tmc2209::Reg::GSTAT, gstat, 50);
        
        if (found) {
            log_info("  [ID %d] FOUND - GSTAT: 0x%02X", id, (unsigned int)gstat & 0xFF);
            
            // Versuche auch IOIN zu lesen für weitere Infos
            uint32_t ioin = 0;
            if (test_driver.readRegister(tmc2209::Reg::IOIN, ioin, 50)) {
                log_info("         IOIN: 0x%08X", (unsigned int)ioin);
            }
        } else {
            log_info("  [ID %d] Not found", id);
        }
    }
    
}
single_led::M<false> led(gpio::Pin::PB12);
single_led::BlinkPattern blink_pattern(200, 800);

// Setup-Funktion: einmalig beim Start
extern "C" void app_setup(void) {
    // Begrüßung senden
    log_info("=== STM32 C++ Application Started ===");
    log_info(u8"Wi‑Fi 📶  Batterie 🔋  Rakete 🚀");
    gpio::Gpio::ConfigureGPIOOutput(gpio::Pin::PB12, false);
    
    // TMC2209 Funktionstest
    test_tmc2209_devices();
    led.AnimatePixel(HAL_GetTick(), &blink_pattern);
    
    last_tick = HAL_GetTick();
}

// Loop-Funktion: im Hauptloop aufgerufen
extern "C" void app_loop(void) {
    uint32_t current_tick = HAL_GetTick();
    led.Loop(current_tick);
    // Jede Sekunde ausführen
    if ((current_tick - last_tick) >= LOOP_INTERVAL_MS) {
        last_tick = current_tick;
        loop_count++;
        
        // LED an PB12 umschalten
        
        
        // Ausgabe an USART2
        log_info("[%d] LED toggle at PB12", loop_count);
    }
}
