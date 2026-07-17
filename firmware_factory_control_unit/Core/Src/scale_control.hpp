#pragma once
// Waegezelle A (HX711). setup() einmalig, update() einmal pro io_thread-Zyklus (s. io_thread.cpp).
void scale_setup();
void scale_update();
