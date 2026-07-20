// Ersetzt die separate TIM6-basierte HAL-Zeitbasis (stm32h5xx_hal_timebase_tim.c -- CubeMX
// generiert diese Datei inzwischen gar nicht mehr, seit die .ioc-Zeitbasis ueber die CubeMX-GUI
// auf SysTick umgestellt wurde) durch eine Umleitung auf ThreadX (tx_time_get()/
// tx_thread_sleep(), sobald der Scheduler laeuft) bzw. den DWT-Zykluszaehler als Fallback (bevor
// der Scheduler laeuft, oder aus einer ISR heraus -- tx_thread_sleep() ist dort nicht zulaessig).
// Grund fuer diese Datei trotz SysTick-Zeitbasis in der .ioc: SysTick selbst wird exklusiv von
// ThreadX fuer den eigenen Scheduler-Tick beansprucht (s. tx_initialize_low_level.S) -- CubeMX'
// eigene, SysTick-basierte Default-HAL_InitTick()-Implementierung wuerde denselben Interrupt-
// Vektor beanspruchen wollen, der bereits per SysTick_Handler-Alias (s. stm32h5xx_it.c) an
// ThreadX vergeben ist. TIM6 ist dadurch komplett unbenutzt -- frei fuer eine kuenftige eigene
// Verwendung.
//
// HAL_InitTick()/HAL_GetTick()/HAL_Delay()/HAL_SuspendTick()/HAL_ResumeTick() sind in
// stm32h5xx_hal.c allesamt als __weak deklariert -- die folgenden starken Definitionen
// ueberschreiben sie automatisch beim Linken, ganz ohne CubeMX-Beteiligung (wie schon bei
// SVC_Handler/PendSV_Handler/SysTick_Handler in stm32h5xx_it.c und bei USB_DRD_FS_IRQHandler in
// usbd_device.c).
#include "main.h"
#include "hal_tick_threadx.h"
#include "tx_api.h"
#include <stdbool.h>

static volatile bool g_threadx_ticking = false;

void hal_tick_threadx_mark_running(void) {
    g_threadx_ticking = true;
}

// Cycles-seit-Boot in Millisekunden, ueber die aktuelle (SystemCoreClockUpdate()-gepflegte,
// s. HAL_RCC_GetHCLKFreq()) Taktfrequenz umgerechnet. Ueberlauf alle ~26.8s bei 160MHz
// (32-Bit-Zaehler) -- fuer die kurzen Zeitspannen, in denen diese Funktion tatsaechlich als
// Fallback dient (vor ThreadX-Start bzw. aus einer ISR heraus), unkritisch.
static uint32_t dwt_millis(void) {
    return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000U));
}

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
    (void)TickPriority;
    // Nichts zu tun -- ThreadX' eigene _tx_initialize_low_level() (s.
    // tx_initialize_low_level.S) konfiguriert SysTick vollstaendig selbst; der DWT-Zykluszaehler
    // wird bereits ganz am Anfang von main() aktiviert (USER CODE BEGIN 1), lange bevor
    // HAL_Init() (und damit dieser Aufruf) ueberhaupt laeuft.
    return HAL_OK;
}

void HAL_SuspendTick(void) {
    // Kein separater Timer-Interrupt mehr, den man anhalten koennte -- nichts zu tun.
}

void HAL_ResumeTick(void) {
    // Siehe HAL_SuspendTick().
}

uint32_t HAL_GetTick(void) {
    if (g_threadx_ticking) {
        // TX_TIMER_TICKS_PER_SECOND=100 -> exakt 10ms/Tick, keine Rundungsverluste. Aufloesung
        // damit 10ms statt der sonst ueblichen 1ms -- fuer die HAL_Delay()-Aufrufe in diesem
        // Projekt (Reset-Pulse, Sensor-Anlaufzeiten im Millisekundenbereich) unkritisch.
        return (uint32_t)(tx_time_get() * (1000UL / TX_TIMER_TICKS_PER_SECOND));
    }
    return dwt_millis();
}

void HAL_Delay(uint32_t Delay) {
    // tx_thread_sleep() ist nur aus einem laufenden ThreadX-Thread-Kontext heraus zulaessig --
    // weder aus einer ISR (__get_IPSR()!=0) noch bevor der Scheduler ueberhaupt Threads
    // ausfuehrt (g_threadx_ticking==false, s. hal_tick_threadx_mark_running()). In beiden
    // Faellen stattdessen per DWT busy-warten (immer verfuegbar, nie unzulaessig).
    if (g_threadx_ticking && (__get_IPSR() == 0U)) {
        // Aufrunden (nie kuerzer schlafen als angefragt): ceil(Delay * TX_TIMER_TICKS_PER_SECOND
        // / 1000). ThreadX' tx_thread_sleep(N) garantiert ohnehin "mindestens N Ticks" (in der
        // Praxis oft N+1, s. Debugging-Sitzung) -- das passt genau zur gewuenschten "im Zweifel
        // laenger warten"-Semantik von HAL_Delay(), daher hier bewusst KEINE zusaetzliche
        // Kompensation dieses Verhaltens (anders als bei der Heartbeat-Ausrichtung auf ein
        // exaktes Ziel-Tick, s. App::HeartbeatThread()).
        ULONG ticks = (ULONG)(((unsigned long long)Delay * TX_TIMER_TICKS_PER_SECOND + 999UL) / 1000UL);
        if (ticks > 0U) {
            tx_thread_sleep(ticks);
        }
    } else {
        uint32_t start = DWT->CYCCNT;
        uint32_t cycles_needed = Delay * (SystemCoreClock / 1000U);
        while ((uint32_t)(DWT->CYCCNT - start) < cycles_needed) {
            // Busy-Wait -- die (uint32_t)-Subtraktion ist ueberlaufsicher, solange Delay
            // deutlich unter der DWT-Wraparound-Periode (~26.8s bei 160MHz) bleibt.
        }
    }
}
