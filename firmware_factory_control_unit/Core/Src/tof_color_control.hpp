#pragma once
// 3x VL53L0X + 1x TCS34725. setup() einmalig, update() einmal pro io_thread-Zyklus (s.
// io_thread.cpp). ReadRangeMm() kann pro Sensor bis zu 50ms blockieren (Timeout-Default) --
// im ungeguenstigsten Fall (alle drei ToF-Sensoren gleichzeitig langsam) verlaengert das den
// io_thread-Zyklus entsprechend; ein fehlender/nicht antwortender Sensor faellt dagegen
// schnell durch (I2C-NACK, nicht das volle Timeout).
void tof_color_setup();
void tof_color_update();
