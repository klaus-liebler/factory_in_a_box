/* NetX Duo/ThreadX EventLoop implementation -- structurally mirrors
 * libs/open62541/arch/eventloop_posix.c (timers via the shared arch/common/ua_timer.c,
 * delayed-callback list, EventSource registration, UA_EventLoopState lifecycle) but WITHOUT
 * any socket polling: this application's only ConnectionManager (connectionmanager_tcp_netxduo.c)
 * delivers network events out-of-band, directly from NX_TCPSERVER's own worker thread, not
 * from inside run(). See open62541_netxduo_arch.h for the resulting two-thread model.
 *
 * Locking: UA_MULTITHREADING is 0 for this build (UA_ARCHITECTURE=none defaults it, and
 * nothing here overrides it), so UA_LOCK/UA_UNLOCK (config.h.in) compile to no-ops and
 * UA_Lock isn't even typedef'd -- open62541's own substructures (e.g. the UA_Timer embedded
 * below) assume a single caller and do no internal locking of their own.
 *
 * That does NOT mean this port is single-threaded, though: two independent ThreadX threads
 * touch the server (the NX_TCPSERVER worker thread for network events, and the application's
 * "OPC UA pump" thread running UA_Server_run_iterate() to service timers). This file provides
 * the missing serialization itself, as a REAL recursive TX_MUTEX behind eventLoop->lock()/
 * unlock() -- which is exactly what open62541's own lockServer()/unlockServer()
 * (src/server/ua_server.c) already calls around every service entry point and around every
 * network callback dispatch (see ua_server_binary.c serverNetworkCallback). run() itself also
 * takes this lock while processing due timers/delayed callbacks, mirroring
 * UA_EventLoopPOSIX_run(); connectionmanager_tcp_netxduo.c's NX_TCPSERVER trampolines take it
 * too, mirroring how POSIX's run() holds its lock for the whole (locked) pollFDs() dispatch. */
#include "open62541_netxduo_arch.h"
#include "eventloop_common.h"
#include "ua_timer.h"
#include "tx_api.h"

typedef struct {
    UA_EventLoop eventLoop;
    UA_Timer timer;
    UA_DelayedCallback *delayedCallbacks;
    TX_MUTEX elMutex;
    UA_Boolean executing;
} UA_EventLoopNX;

/*********/
/* Clock */
/*********/

static UA_DateTime
NX_DateTime_now(UA_EventLoop *el) {
    (void)el;
    return UA_DateTime_now();
}

static UA_DateTime
NX_DateTime_nowMonotonic(UA_EventLoop *el) {
    (void)el;
    return UA_DateTime_nowMonotonic();
}

static UA_Int64
NX_DateTime_localTimeUtcOffset(UA_EventLoop *el) {
    (void)el;
    return UA_DateTime_localTimeUtcOffset();
}

/*********/
/* Timer */
/*********/

static UA_DateTime
NX_nextCyclicTime(UA_EventLoop *public_el) {
    UA_EventLoopNX *el = (UA_EventLoopNX*)public_el;
    if(el->delayedCallbacks)
        return el->eventLoop.dateTime_nowMonotonic(&el->eventLoop);
    return UA_Timer_nextRepeatedTime(&el->timer);
}

static UA_StatusCode
NX_addTimedCallback(UA_EventLoop *public_el, UA_Callback callback,
                    void *application, void *data, UA_DateTime date,
                    UA_UInt64 *callbackId) {
    UA_EventLoopNX *el = (UA_EventLoopNX*)public_el;
    return UA_Timer_addTimedCallback(&el->timer, callback, application,
                                     data, date, callbackId);
}

static UA_StatusCode
NX_addCyclicCallback(UA_EventLoop *public_el, UA_Callback cb,
                     void *application, void *data, UA_Double interval_ms,
                     UA_DateTime *baseTime, UA_TimerPolicy timerPolicy,
                     UA_UInt64 *callbackId) {
    UA_EventLoopNX *el = (UA_EventLoopNX*)public_el;
    return UA_Timer_addRepeatedCallback(&el->timer, cb, application,
                                        data, interval_ms, baseTime,
                                        timerPolicy, callbackId);
}

static UA_StatusCode
NX_modifyCyclicCallback(UA_EventLoop *public_el, UA_UInt64 callbackId,
                        UA_Double interval_ms, UA_DateTime *baseTime,
                        UA_TimerPolicy timerPolicy) {
    UA_EventLoopNX *el = (UA_EventLoopNX*)public_el;
    return UA_Timer_changeRepeatedCallback(&el->timer, callbackId,
                                           interval_ms, baseTime, timerPolicy);
}

static void
NX_removeCyclicCallback(UA_EventLoop *public_el, UA_UInt64 callbackId) {
    UA_EventLoopNX *el = (UA_EventLoopNX*)public_el;
    UA_Timer_removeCallback(&el->timer, callbackId);
}

static void
NX_addDelayedCallback(UA_EventLoop *public_el, UA_DelayedCallback *dc) {
    UA_EventLoopNX *el = (UA_EventLoopNX*)public_el;
    tx_mutex_get(&el->elMutex, TX_WAIT_FOREVER);
    dc->next = el->delayedCallbacks;
    el->delayedCallbacks = dc;
    tx_mutex_put(&el->elMutex);
}

static void
NX_removeDelayedCallback(UA_EventLoop *public_el, UA_DelayedCallback *dc) {
    UA_EventLoopNX *el = (UA_EventLoopNX*)public_el;
    tx_mutex_get(&el->elMutex, TX_WAIT_FOREVER);
    UA_DelayedCallback **prev = &el->delayedCallbacks;
    while(*prev) {
        if(*prev == dc) {
            *prev = (*prev)->next;
            break;
        }
        prev = &(*prev)->next;
    }
    tx_mutex_put(&el->elMutex);
}

/* Empties the linked list before processing so a delayed callback can (re-)add itself --
 * that re-added entry is then only processed on the NEXT iteration. Called with elMutex
 * already held (from NX_run()). */
static void
ProcessDelayed(UA_EventLoopNX *el) {
    UA_DelayedCallback *dc = el->delayedCallbacks, *next = NULL;
    el->delayedCallbacks = NULL;
    for(; dc; dc = next) {
        next = dc->next;
        if(!dc->callback)
            continue;
        dc->callback(dc->application, dc->context);
    }
}

/***********************/
/* EventLoop Lifecycle */
/***********************/

static UA_StatusCode
NX_start(UA_EventLoopNX *el) {
    tx_mutex_get(&el->elMutex, TX_WAIT_FOREVER);

    if(el->eventLoop.state != UA_EVENTLOOPSTATE_FRESH &&
       el->eventLoop.state != UA_EVENTLOOPSTATE_STOPPED) {
        tx_mutex_put(&el->elMutex);
        return UA_STATUSCODE_BADINTERNALERROR;
    }

    UA_StatusCode res = UA_STATUSCODE_GOOD;
    UA_EventSource *es = el->eventLoop.eventSources;
    while(es) {
        res |= es->start(es);
        es = es->next;
    }

    *(UA_EventLoopState*)(uintptr_t)&el->eventLoop.state = UA_EVENTLOOPSTATE_STARTED;

    tx_mutex_put(&el->elMutex);
    return res;
}

/* Called with elMutex already held */
static void
CheckClosed(UA_EventLoopNX *el) {
    UA_EventSource *es = el->eventLoop.eventSources;
    while(es) {
        if(es->state != UA_EVENTSOURCESTATE_STOPPED)
            return;
        es = es->next;
    }
    if(el->delayedCallbacks != NULL)
        return;
    *(UA_EventLoopState*)(uintptr_t)&el->eventLoop.state = UA_EVENTLOOPSTATE_STOPPED;
}

static void
NX_stop(UA_EventLoopNX *el) {
    tx_mutex_get(&el->elMutex, TX_WAIT_FOREVER);

    if(el->eventLoop.state != UA_EVENTLOOPSTATE_STARTED) {
        tx_mutex_put(&el->elMutex);
        return;
    }

    *(UA_EventLoopState*)(uintptr_t)&el->eventLoop.state = UA_EVENTLOOPSTATE_STOPPING;

    UA_EventSource *es = el->eventLoop.eventSources;
    for(; es; es = es->next) {
        if(es->state == UA_EVENTSOURCESTATE_STARTING ||
           es->state == UA_EVENTSOURCESTATE_STARTED) {
            es->stop(es);
        }
    }

    CheckClosed(el);

    tx_mutex_put(&el->elMutex);
}

static UA_StatusCode
NX_run(UA_EventLoopNX *el, UA_UInt32 timeout) {
    tx_mutex_get(&el->elMutex, TX_WAIT_FOREVER);

    if(el->executing) {
        tx_mutex_put(&el->elMutex);
        return UA_STATUSCODE_BADINTERNALERROR;
    }
    el->executing = true;

    if(el->eventLoop.state == UA_EVENTLOOPSTATE_FRESH ||
       el->eventLoop.state == UA_EVENTLOOPSTATE_STOPPED) {
        el->executing = false;
        tx_mutex_put(&el->elMutex);
        return UA_STATUSCODE_BADINTERNALERROR;
    }

    UA_DateTime dateBefore = el->eventLoop.dateTime_nowMonotonic(&el->eventLoop);
    UA_DateTime dateNext = UA_Timer_process(&el->timer, dateBefore);
    ProcessDelayed(el);

    /* A delayed callback might have re-added itself (or a new one) -- don't sleep, process
     * it on the next NX_run() call instead (the pump thread's loop calls us again right
     * away). */
    if(el->delayedCallbacks != NULL)
        timeout = 0;

    UA_DateTime maxDate = dateBefore + ((UA_DateTime)timeout * UA_DATETIME_MSEC);
    if(dateNext > maxDate)
        dateNext = maxDate;

    if(el->eventLoop.state == UA_EVENTLOOPSTATE_STOPPING)
        CheckClosed(el);

    el->executing = false;
    tx_mutex_put(&el->elMutex);

    /* Deliberately sleep WITHOUT holding elMutex, unlike the POSIX reference implementation
     * (UA_EventLoopPOSIX_run holds its lock across the whole blocking poll()/epoll_wait(),
     * because THAT wait is also how it notices socket activity). Here socket activity is
     * delivered out-of-band by the NX_TCPSERVER worker thread -- holding the lock across a
     * sleep would block that thread from dispatching new connections/messages for the whole
     * sleep duration. */
    UA_DateTime now = el->eventLoop.dateTime_nowMonotonic(&el->eventLoop);
    UA_DateTime remaining = dateNext - now;
    if(remaining > 0) {
        UA_DateTime hundredNsPerTick = UA_DATETIME_SEC / TX_TIMER_TICKS_PER_SECOND;
        ULONG ticks = (ULONG)(remaining / hundredNsPerTick);
        if(ticks == 0)
            ticks = 1;
        /* Cap: bounds worst-case reaction latency to freshly registered cyclic callbacks
         * and keeps the math above safe regardless of how large "remaining" gets (e.g. when
         * nothing at all is scheduled, UA_Timer_nextRepeatedTime() returns UA_INT64_MAX). */
        if(ticks > TX_TIMER_TICKS_PER_SECOND)
            ticks = TX_TIMER_TICKS_PER_SECOND;
        tx_thread_sleep(ticks);
    }

    return UA_STATUSCODE_GOOD;
}

/*****************************/
/* Registering Event Sources */
/*****************************/

static UA_StatusCode
NX_registerEventSource(UA_EventLoopNX *el, UA_EventSource *es) {
    tx_mutex_get(&el->elMutex, TX_WAIT_FOREVER);

    if(es->state != UA_EVENTSOURCESTATE_FRESH) {
        tx_mutex_put(&el->elMutex);
        return UA_STATUSCODE_BADINTERNALERROR;
    }

    es->next = el->eventLoop.eventSources;
    el->eventLoop.eventSources = es;
    es->eventLoop = &el->eventLoop;
    es->state = UA_EVENTSOURCESTATE_STOPPED;

    UA_StatusCode res = UA_STATUSCODE_GOOD;
    if(el->eventLoop.state == UA_EVENTLOOPSTATE_STARTED)
        res = es->start(es);

    tx_mutex_put(&el->elMutex);
    return res;
}

static UA_StatusCode
NX_deregisterEventSource(UA_EventLoopNX *el, UA_EventSource *es) {
    tx_mutex_get(&el->elMutex, TX_WAIT_FOREVER);

    if(es->state != UA_EVENTSOURCESTATE_STOPPED) {
        tx_mutex_put(&el->elMutex);
        return UA_STATUSCODE_BADINTERNALERROR;
    }

    UA_EventSource **s = &el->eventLoop.eventSources;
    while(*s) {
        if(*s == es) {
            *s = es->next;
            break;
        }
        s = &(*s)->next;
    }
    es->state = UA_EVENTSOURCESTATE_FRESH;

    tx_mutex_put(&el->elMutex);
    return UA_STATUSCODE_GOOD;
}

/*************************/
/* Initialize and Delete */
/*************************/

static UA_StatusCode
NX_free(UA_EventLoopNX *el) {
    tx_mutex_get(&el->elMutex, TX_WAIT_FOREVER);

    if(el->eventLoop.state != UA_EVENTLOOPSTATE_STOPPED &&
       el->eventLoop.state != UA_EVENTLOOPSTATE_FRESH) {
        tx_mutex_put(&el->elMutex);
        return UA_STATUSCODE_BADINTERNALERROR;
    }

    while(el->eventLoop.eventSources) {
        UA_EventSource *es = el->eventLoop.eventSources;
        NX_deregisterEventSource(el, es);
        es->free(es);
    }

    UA_Timer_clear(&el->timer);
    ProcessDelayed(el);

    tx_mutex_put(&el->elMutex);
    tx_mutex_delete(&el->elMutex);
    UA_free(el);
    return UA_STATUSCODE_GOOD;
}

static void
NX_lock(UA_EventLoop *public_el) {
    UA_EventLoopNX *el = (UA_EventLoopNX*)public_el;
    tx_mutex_get(&el->elMutex, TX_WAIT_FOREVER);
}

static void
NX_unlock(UA_EventLoop *public_el) {
    UA_EventLoopNX *el = (UA_EventLoopNX*)public_el;
    tx_mutex_put(&el->elMutex);
}

UA_EventLoop *
UA_EventLoop_new_NetXDuo(const UA_Logger *logger) {
    UA_EventLoopNX *el = (UA_EventLoopNX*)UA_calloc(1, sizeof(UA_EventLoopNX));
    if(!el)
        return NULL;

    /* TX_INHERIT: avoids priority-inversion stalls between the (typically lower-priority)
     * OPC UA pump thread and the (typically higher-priority, reacting to live network
     * traffic) NX_TCPSERVER worker thread -- same rationale as this project's other
     * cross-cutting mutexes, e.g. app.cc's malloc_mutex. */
    tx_mutex_create(&el->elMutex, (CHAR*)(uintptr_t)"OPC UA EventLoop Mutex", TX_INHERIT);
    UA_Timer_init(&el->timer);

    el->eventLoop.logger = logger;

    el->eventLoop.start = (UA_StatusCode (*)(UA_EventLoop*))NX_start;
    el->eventLoop.stop = (void (*)(UA_EventLoop*))NX_stop;
    el->eventLoop.run = (UA_StatusCode (*)(UA_EventLoop*, UA_UInt32))NX_run;
    el->eventLoop.free = (UA_StatusCode (*)(UA_EventLoop*))NX_free;

    el->eventLoop.dateTime_now = NX_DateTime_now;
    el->eventLoop.dateTime_nowMonotonic = NX_DateTime_nowMonotonic;
    el->eventLoop.dateTime_localTimeUtcOffset = NX_DateTime_localTimeUtcOffset;

    el->eventLoop.nextCyclicTime = NX_nextCyclicTime;
    el->eventLoop.addCyclicCallback = NX_addCyclicCallback;
    el->eventLoop.modifyCyclicCallback = NX_modifyCyclicCallback;
    el->eventLoop.removeCyclicCallback = NX_removeCyclicCallback;
    el->eventLoop.addTimedCallback = NX_addTimedCallback;
    el->eventLoop.addDelayedCallback = NX_addDelayedCallback;
    el->eventLoop.removeDelayedCallback = NX_removeDelayedCallback;

    el->eventLoop.registerEventSource =
        (UA_StatusCode (*)(UA_EventLoop*, UA_EventSource*))NX_registerEventSource;
    el->eventLoop.deregisterEventSource =
        (UA_StatusCode (*)(UA_EventLoop*, UA_EventSource*))NX_deregisterEventSource;

    el->eventLoop.lock = NX_lock;
    el->eventLoop.unlock = NX_unlock;

    return &el->eventLoop;
}
