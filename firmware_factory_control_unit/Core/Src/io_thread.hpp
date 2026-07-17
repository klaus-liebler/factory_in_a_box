#pragma once
#include "tx_api.h"

// Groesserer Stack als vorher: dieser eine Thread deckt jetzt alle Subsysteme ab (digitale
// I/O, LED, Ethernet-Link, USB-PD, Waegezelle, Stepper-Register, ToF/Farbsensor, CAN,
// WS2812), statt jeweils einen eigenen kleinen Stack pro Thread zu haben.
constexpr ULONG IO_THREAD_STACK_SIZE = 6 * 1024;
constexpr UINT IO_THREAD_PRIORITY = 12;
// 5 Ticks * 10ms/Tick (TX_TIMER_TICKS_PER_SECOND=100, s. tx_user.h) = 50ms.
constexpr ULONG IO_THREAD_SLEEP_TICKS = 5;

// Einmalige Registerlevel-Initialisierung aller Subsysteme (ADC-Start, PWM-Modus-Fixups,
// TMC2209-UART-Init, I2C-Sensor-Init, FDCAN-Filter, WS2812/LED-Init, USB-PD-Start).
//
// Wird als extern "C" definiert (s. io_thread.cpp) und aus main() heraus VOR
// tx_kernel_enter() aufgerufen (s. main.c, USER CODE 2), nicht aus tx_application_define():
// mehrere der hier aufgerufenen *_setup()-Funktionen (tof_color_setup(), stepper_setup())
// nutzen HAL_Delay()/HAL-Timeouts (HAL_I2C_*/HAL_UART_* mit endlichem Timeout-Parameter), die
// sich waehrend tx_application_define() aufhaengen wuerden -- tx_kernel_enter() sperrt
// Interrupts fuer die Dauer dieses Aufrufs, HAL_GetTick() zaehlt dann nicht mehr hoch, und
// jeder darauf basierende Timeout-Vergleich wird nie wahr.
extern "C" void io_setup();

void io_thread_entry(ULONG arg);
