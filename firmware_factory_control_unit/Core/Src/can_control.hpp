#pragma once
// FDCAN1-Basisstatistik. setup() einmalig, update() einmal pro io_thread-Zyklus (s.
// io_thread.cpp).
void can_setup();
void can_update();
