#pragma once
// UART-Konfiguration der TMC2209-Treiber-ICs + NVIC/Timer-Init fuer die Motion-Erzeugung.
// setup() einmalig, update() einmal pro io_thread-Zyklus (s. io_thread.cpp). Die eigentliche
// STEP/DIR-Pulserzeugung laeuft unabhaengig vom io_thread per Timer-Update-Interrupt (siehe
// stepper_control.cpp, TIM16_IRQHandler/TIM17_IRQHandler).
void stepper_setup();
void stepper_update();
