/**
 * @file malloc_lock.c
 * @brief Thread-safe newlib malloc/new hooks for ThreadX (__malloc_lock/__malloc_unlock)
 *
 * newlib-nano's malloc calls __malloc_lock()/__malloc_unlock() (weak, no-op by default)
 * around every allocation. Without an override, malloc/new/STL containers are not safe
 * to use from more than one ThreadX thread at a time. malloc_lock_init() must be called
 * once, before any thread other than the caller can run (e.g. from tx_application_define,
 * before threads are created) - calls that happen earlier are inherently single-threaded
 * and are safely no-ops.
 */

#include "tx_api.h"

static TX_MUTEX malloc_mutex;
static volatile int malloc_mutex_ready = 0;

void malloc_lock_init(void)
{
    tx_mutex_create(&malloc_mutex, "malloc_mutex", TX_INHERIT);
    malloc_mutex_ready = 1;
}

void __malloc_lock(struct _reent *reent)
{
    (void)reent;
    if (malloc_mutex_ready)
    {
        tx_mutex_get(&malloc_mutex, TX_WAIT_FOREVER);
    }
}

void __malloc_unlock(struct _reent *reent)
{
    (void)reent;
    if (malloc_mutex_ready)
    {
        tx_mutex_put(&malloc_mutex);
    }
}
