/* NetX Duo/ThreadX port of open62541's clock.c (see e.g. libs/open62541/arch/posix/ua_clock.c
 * for the reference this mirrors). Declarations come from open62541/types.h, unguarded by any
 * UA_ARCHITECTURE_* macro -- every architecture must provide these three. */
#include <open62541/types.h>
#include "tx_api.h"

/* This board has no RTC and no SNTP client (see project notes) -- there is no wall-clock time
 * source at all. SecurityPolicy None (the only policy this server supports) never validates
 * certificate validity periods against UA_DateTime_now(), so an arbitrary epoch offset is
 * acceptable: tx_time_get()==0 (i.e. boot) is mapped to the Unix epoch. The result is
 * monotonic and increases at the correct rate -- it just doesn't reflect the real date. */

static UA_DateTime
TicksToHundredNs(ULONG ticks) {
    /* UA_DateTime is 100ns units; multiply before dividing to keep precision (ticks is 32-bit,
     * UA_DATETIME_SEC is 10,000,000 -- the product comfortably fits UA_DateTime's 64 bits). */
    return ((UA_DateTime)ticks * UA_DATETIME_SEC) / TX_TIMER_TICKS_PER_SECOND;
}

UA_DateTime
UA_DateTime_now(void) {
    return TicksToHundredNs(tx_time_get()) + UA_DATETIME_UNIX_EPOCH;
}

UA_Int64
UA_DateTime_localTimeUtcOffset(void) {
    /* No timezone database available -- treat local time as UTC. */
    return 0;
}

UA_DateTime
UA_DateTime_nowMonotonic(void) {
    return TicksToHundredNs(tx_time_get());
}
