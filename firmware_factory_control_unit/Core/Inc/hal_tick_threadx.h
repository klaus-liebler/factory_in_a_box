#pragma once
// Deklaration fuer hal_tick_threadx.c (s. dort fuer die volle Begruendung): biegt
// HAL_GetTick()/HAL_Delay()/HAL_InitTick()/HAL_Suspend-ResumeTick() auf ThreadX (tx_time_get()/
// tx_thread_sleep()) bzw. den DWT-Zykluszaehler um, statt TIM6 als separate HAL-Zeitbasis zu
// benutzen -- TIM6 ist dadurch wieder fuer andere Zwecke frei.
#ifdef __cplusplus
extern "C" {
#endif

// Muss GENAU EINMAL aufgerufen werden, sobald der ThreadX-Scheduler tatsaechlich einen
// normalen Thread ausfuehrt (z.B. als allererste Zeile in App::AppThread(), dem einzigen
// TX_AUTO_START-Thread) -- NICHT aus tx_application_define()/vor tx_kernel_enter() heraus, da
// dort noch kein echter Thread-Kontext existiert und tx_thread_sleep() dort undefiniert waere.
// Vorher (Boot, App::SetupBeforeThreadX(), etc.) liefert tx_time_get() nur eine konstante 0 --
// HAL_GetTick()/HAL_Delay() weichen bis zu diesem Aufruf automatisch auf den DWT-Zykluszaehler
// aus (von main.c ganz am Boot-Anfang aktiviert).
void hal_tick_threadx_mark_running(void);

#ifdef __cplusplus
}
#endif
