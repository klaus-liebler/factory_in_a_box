#pragma once
// ThreadX' offizielles Execution-Profiling-Feature (TX_EXECUTION_PROFILE_ENABLE, s. tx_user.h)
// erwartet genau diese Datei plus eine Implementierung von _tx_execution_thread_enter()/
// _tx_execution_thread_exit() (s. tx_execution_profile.c) -- beides ist im vendorten ThreadX-Paket
// (libs/ST/threadx) NICHT enthalten, nur die Aufrufstellen dafuer in
// libs/ST/threadx/ports/cortex_m33/gnu/src/tx_thread_schedule.S und das davon befuellte
// TX_THREAD-Feld tx_thread_execution_time_total (s. tx_api.h). Diese Datei ergaenzt das Fehlende,
// ohne den vendorten ThreadX-Code selbst anzufassen.
#include "tx_port.h"

typedef ULONG EXECUTION_TIME;
typedef ULONG EXECUTION_TIME_SOURCE_TYPE;

#ifdef __cplusplus
extern "C" {
#endif

VOID _tx_execution_thread_enter(VOID);
VOID _tx_execution_thread_exit(VOID);

#ifdef __cplusplus
}
#endif
