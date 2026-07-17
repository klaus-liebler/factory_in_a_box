#pragma once
// WS2812_1/WS2812_2 (PE5/PE6, TIM15 CH1/CH2). setup() einmalig, update() einmal pro
// io_thread-Zyklus (s. io_thread.cpp).
void ws2812_setup();
void ws2812_update();
