#pragma once
// INFOLED (PE15, aktiv=LOW) ueber stm32_libs/common_stm32/single_led.hh. init() einmalig,
// update() einmal pro io_thread-Zyklus (s. io_thread.cpp).
void led_control_init();
void led_control_update();
