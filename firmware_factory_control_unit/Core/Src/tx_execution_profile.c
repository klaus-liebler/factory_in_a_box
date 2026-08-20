// Implementierung der beiden Hooks, die tx_thread_schedule.S bei jedem Kontextwechsel aufruft,
// sobald TX_EXECUTION_PROFILE_ENABLE definiert ist (s. tx_user.h) -- s. Klassenkommentar in
// tx_execution_profile.h zur Begruendung, warum diese Datei ueberhaupt noetig ist (im vendorten
// ThreadX-Paket fehlend). Zeitquelle ist der DWT-Zykluszaehler (DWT->CYCCNT), der bereits vor
// ThreadX-Start laeuft (s. hal_tick_threadx.c) -- dadurch funktioniert das Timing auch fuer den
// allerersten Thread-Wechsel, ganz ohne Sonderfall.
//
// _tx_thread_current_ptr zeigt beim Aufruf von _tx_execution_thread_exit() noch auf den Thread,
// der GERADE VERDRAENGT wird (tx_thread_schedule.S ruft diesen Hook vor der eigentlichen
// Kontext-Sicherung auf), und bei _tx_execution_thread_enter() bereits auf den NEUEN Thread (der
// Scheduler traegt _tx_thread_current_ptr vorher ein) -- exakt das Verhalten, das
// tx_thread_execution_time_total braucht: Ende-minus-Start akkumuliert pro Thread nur dessen
// eigene Laufzeit.
//
// uint32_t-Ueberlauf (DWT->CYCCNT wraprt alle ~26.8s bei 160MHz, tx_thread_execution_time_total
// entsprechend bei Akkumulation ueber die Betriebszeit) ist unkritisch: alle Differenzen, die
// task_monitor.cpp daraus bildet, sind unsigned-Subtraktionen ueber kurze Intervalle -- dieselbe
// Ueberlauf-sichere Modulo-Arithmetik wie bei tx_time_get()-Deltas andernorts im Projekt (s. z.B.
// hal_tick_threadx.c).
#include "tx_execution_profile.h"
#include "tx_api.h"
#include "main.h"

extern TX_THREAD *_tx_thread_current_ptr;

// Von _tx_initialize_kernel_enter() einmalig vor dem allerersten Scheduler-Lauf aufgerufen (s.
// tx_initialize_kernel_enter.c) -- nichts zu tun: es gibt keinen globalen Profiling-Zustand,
// jedes TX_THREAD traegt sein tx_thread_execution_time_total/_last_start bereits selbst (per
// tx_thread_create() auf 0 initialisiert), und die Zeitquelle (DWT->CYCCNT) laeuft ohnehin schon.
VOID _tx_execution_initialize(VOID) {
}

VOID _tx_execution_thread_exit(VOID) {
    TX_THREAD *t = _tx_thread_current_ptr;
    if (t != TX_NULL) {
        t->tx_thread_execution_time_total += (ULONG)(DWT->CYCCNT - t->tx_thread_execution_time_last_start);
    }
}

VOID _tx_execution_thread_enter(VOID) {
    TX_THREAD *t = _tx_thread_current_ptr;
    if (t != TX_NULL) {
        t->tx_thread_execution_time_last_start = DWT->CYCCNT;
    }
}
