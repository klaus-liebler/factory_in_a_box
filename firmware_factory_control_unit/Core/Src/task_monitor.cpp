// Beantwortet tasks.TaskManagerRequest mit einer tasks.TaskListMessage -- ein Snapshot aller
// App-eigenen ThreadX-Threads (Name/Zustand/Prioritaet/Stack-Verbrauch inkl. High-Water-Mark/
// CPU-Last), gezielt an genau die anfragende Verbindung gesendet (s. Aufrufer in
// webserver.cpp::handle_ws_message()) -- bewusst KEIN web_server.Broadcast(): andere verbundene
// Clients, die die Task-Manager-Seite gar nicht offen haben (s. web/src/apps/task-manager-app.ts),
// sollen dafuer nicht mitbezahlen. Wird dementsprechend auch nur dann ueberhaupt aufgerufen, wenn
// im Browser tatsaechlich eine TaskManagerRequest ankommt -- kein eigener Thread, kein Timer.
#include "task_monitor.hpp"
#include "app.hh"
#include "generated/ws_protocol.hh"
#include "tx_api.h"
#include "main.h"
#include <cstring>

// _tx_thread_stack_analyze() (libs/ST/threadx/common/src/tx_thread_stack_analyze.c) ist keine
// offizielle tx_api.h-Oberflaeche (kein tx_thread_stack_analyze()-Wrapper), aber eine normale
// externe C-Funktion, die immer mituebersetzt wird -- Stack-Filling (0xEF-Muster) ist in diesem
// Projekt per Default aktiv (s. tx_user.h, TX_DISABLE_STACK_FILLING bleibt auskommentiert), daher
// funktioniert die darauf basierende High-Water-Mark-Suche ohne TX_ENABLE_STACK_CHECKING.
extern "C" void _tx_thread_stack_analyze(TX_THREAD *thread_ptr);

namespace {

constexpr size_t TASK_COUNT = 6;

struct ThreadEntry {
    TX_THREAD *thread;
    ULONG prev_execution_time;
};

// Feste Liste statt generischer Enumeration ueber ThreadX' interne verkettete Thread-Liste --
// passt zur aktuellen Groessenordnung von 6 Threads (s. app.hh-Klassenkommentar); ein neuer
// Thread braucht hier eine zusaetzliche Zeile. Function-local static (statt globalem Array): wird
// erst beim ersten tatsaechlichen Request initialisiert, lange nach App::Instance()-Konstruktion,
// vermeidet damit jede Static-Init-Order-Frage.
ThreadEntry *GetThreadEntries() {
    App &app = App::Instance();
    static ThreadEntry entries[TASK_COUNT] = {
        {&app.app_main_thread, 0},
        {&app.modbus_tcp_server_thread, 0},
        {&app.modbus_rtu_thread, 0},
        {&app.io_thread, 0},
        {&app.usbd_device_thread, 0},
        {&app.heartbeat_thread, 0},
    };
    // tx_thread_stack_highest_ptr wird von ThreadX selbst NUR unter TX_ENABLE_STACK_CHECKING bei
    // tx_thread_create() gesetzt (s. tx_thread_create.c: "#ifdef TX_ENABLE_STACK_CHECKING ...
    // tx_thread_stack_highest_ptr = tx_thread_stack_ptr; #endif") -- ohne dieses Flag bleibt das
    // Feld auf 0 (vom TX_MEMSET beim Erzeugen). _tx_thread_stack_analyze()'s Binaersuche haette
    // dann bei ihrem allerersten Aufruf 0 als obere Suchgrenze genommen und eine daraus berechnete,
    // weit ausserhalb des echten Stacks liegende Adresse dereferenziert -- beobachtet als absurde
    // Peak-Werte (mehrere hundert MiB) auf der Task-Manager-Seite. Hier deshalb einmalig genau das
    // nachgeholt, was ThreadX unter TX_ENABLE_STACK_CHECKING ohnehin taete, ohne dessen Laufzeit-
    // Overhead (Ueberlaufpruefung bei jedem Kontextwechsel) zu uebernehmen. Da das Seeding erst
    // beim ersten Request passiert (Threads laufen zu dem Zeitpunkt schon), zaehlt "High-Water-Mark"
    // ab hier ("jetzt"), nicht ab Thread-Erzeugung -- ein vor dem ersten Request erreichter,
    // tieferer Stand bleibt unsichtbar; jeder folgende Request faengt neue Tiefststaende korrekt ein.
    static bool seeded = [&]() {
        for (size_t i = 0; i < TASK_COUNT; i++) {
            entries[i].thread->tx_thread_stack_highest_ptr = entries[i].thread->tx_thread_stack_ptr;
        }
        return true;
    }();
    (void)seeded;
    return entries;
}

// Ein einziger globaler Zeitanker (statt je Thread) genuegt: alle Threads teilen sich dasselbe
// Anfrageintervall, da EIN Request-Handler-Aufruf immer alle 6 Threads gemeinsam neu vermisst.
ULONG g_prev_wall_cycles = 0;
bool g_has_previous_sample = false;

WsProtocol::tasks::ThreadState MapState(UINT tx_state) {
    switch (tx_state) {
        case TX_READY:           return WsProtocol::tasks::ThreadState::Ready;
        case TX_COMPLETED:       return WsProtocol::tasks::ThreadState::Completed;
        case TX_TERMINATED:      return WsProtocol::tasks::ThreadState::Terminated;
        case TX_SUSPENDED:       return WsProtocol::tasks::ThreadState::Suspended;
        case TX_SLEEP:           return WsProtocol::tasks::ThreadState::Sleep;
        case TX_QUEUE_SUSP:      return WsProtocol::tasks::ThreadState::QueueSuspended;
        case TX_SEMAPHORE_SUSP:  return WsProtocol::tasks::ThreadState::SemaphoreSuspended;
        case TX_EVENT_FLAG:      return WsProtocol::tasks::ThreadState::EventFlag;
        case TX_BLOCK_MEMORY:    return WsProtocol::tasks::ThreadState::BlockMemory;
        case TX_BYTE_MEMORY:     return WsProtocol::tasks::ThreadState::ByteMemory;
        case TX_IO_DRIVER:       return WsProtocol::tasks::ThreadState::IoDriver;
        case TX_FILE:            return WsProtocol::tasks::ThreadState::File;
        case TX_TCP_IP:          return WsProtocol::tasks::ThreadState::TcpIp;
        case TX_MUTEX_SUSP:      return WsProtocol::tasks::ThreadState::MutexSuspended;
        case TX_PRIORITY_CHANGE: return WsProtocol::tasks::ThreadState::PriorityChange;
        default:                 return WsProtocol::tasks::ThreadState::Unknown;
    }
}

void FillName(uint8_t out[32], const char *name) {
    memset(out, 0, 32);
    if (name == nullptr) return;
    size_t len = strlen(name);
    if (len > 31) len = 31;
    memcpy(out, name, len);
}

} // namespace

void HandleTaskManagerRequest(Http::WebSocketConnection &conn, uint16_t request_id) {
    ULONG now_cycles = DWT->CYCCNT;
    ULONG delta_wall = g_has_previous_sample ? (ULONG)(now_cycles - g_prev_wall_cycles) : 0;

    ThreadEntry *entries = GetThreadEntries();
    WsProtocol::tasks::TaskInfo items[TASK_COUNT];

    for (size_t i = 0; i < TASK_COUNT; i++) {
        TX_THREAD *t = entries[i].thread;

        CHAR *name = nullptr;
        UINT state = 0, priority = 0, preempt_threshold = 0;
        ULONG run_count = 0, time_slice = 0;
        TX_THREAD *next_thread = nullptr, *next_suspended = nullptr;
        tx_thread_info_get(t, &name, &state, &run_count, &priority, &preempt_threshold,
                            &time_slice, &next_thread, &next_suspended);

        // Aktualisiert tx_thread_stack_highest_ptr per Binaersuche nach dem Fuellmuster --
        // absichtlich JEDES Mal vor dem Auslesen aufgerufen (kein zusaetzlicher periodischer
        // Trigger noetig), da der Aufruf ohnehin nur beim seltenen expliziten Request passiert.
        _tx_thread_stack_analyze(t);

        WsProtocol::tasks::TaskInfo &item = items[i];
        FillName(item.name, name);
        item.state = MapState(state);
        item.priority = (uint8_t)priority;
        item.stackSizeBytes = (uint32_t)t->tx_thread_stack_size;
        // Cortex-M-Stack waechst abwaerts: "Verbrauch" ist der Abstand vom oberen Ende (Start der
        // Nutzung) bis zum jeweiligen (aktuellen bzw. hoechstjemals erreichten) Pointer.
        item.stackUsedCurrentBytes =
            (uint32_t)((uint8_t *)t->tx_thread_stack_end - (uint8_t *)t->tx_thread_stack_ptr);
        item.stackUsedHighWaterBytes =
            (uint32_t)((uint8_t *)t->tx_thread_stack_end - (uint8_t *)t->tx_thread_stack_highest_ptr);
        item.runCount = (uint32_t)run_count;

        ULONG exec_now = t->tx_thread_execution_time_total;
        if (g_has_previous_sample && delta_wall > 0) {
            ULONG delta_exec = (ULONG)(exec_now - entries[i].prev_execution_time);
            uint32_t permille = (uint32_t)(((uint64_t)delta_exec * 1000ULL) / (uint64_t)delta_wall);
            item.cpuPermille = (uint16_t)(permille > 1000 ? 1000 : permille);
        } else {
            // Allererster Request seit Boot -- kein Vorwert vorhanden, s. Klassenkommentar oben.
            item.cpuPermille = 0;
        }
        entries[i].prev_execution_time = exec_now;
    }

    g_prev_wall_cycles = now_cycles;
    g_has_previous_sample = true;

    uint8_t element_buffer[TASK_COUNT * WsProtocol::tasks::TaskListMessageItems_ELEMENT_SIZE];
    size_t element_pos = 0;
    for (size_t i = 0; i < TASK_COUNT; i++) {
        element_pos += WsProtocol::tasks::EncodeTaskListMessageItemsElement(
            items[i], element_buffer + element_pos, sizeof(element_buffer) - element_pos);
    }

    WsProtocol::tasks::TaskListMessage::Payload payload{};
    payload.requestId = request_id;
    payload.itemsData = element_buffer;
    payload.itemsCount = TASK_COUNT;

    uint8_t frame[512];
    size_t encoded = WsProtocol::tasks::TaskListMessage::Encode(payload, frame, sizeof(frame));
    if (encoded > 0) {
        conn.SendBinary(frame, encoded);
    }
}
