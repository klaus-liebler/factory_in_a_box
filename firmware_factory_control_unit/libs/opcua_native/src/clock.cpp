#include "opcua/clock.hpp"

#include "tx_api.h"

namespace opcua {

namespace {
DateTime TicksToHundredNs(ULONG ticks) {
    // Multiply before dividing to keep precision (ticks is 32-bit, DATETIME_SEC is
    // 10,000,000 -- the product comfortably fits DateTime's 64 bits).
    return (static_cast<DateTime>(ticks) * DATETIME_SEC) / TX_TIMER_TICKS_PER_SECOND;
}
} // namespace

DateTime Now() {
    return TicksToHundredNs(tx_time_get()) + DATETIME_UNIX_EPOCH;
}

DateTime NowMonotonic() {
    return TicksToHundredNs(tx_time_get());
}

} // namespace opcua
