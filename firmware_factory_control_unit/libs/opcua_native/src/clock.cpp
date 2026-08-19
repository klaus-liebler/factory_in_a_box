#include "opcua/clock.hpp"

#include "tx_api.h"

namespace opcua {

namespace {
DateTime TicksToHundredNs(ULONG ticks) {
    // Multiply before dividing to keep precision (ticks is 32-bit, DATETIME_SEC is
    // 10,000,000 -- the product comfortably fits DateTime's 64 bits).
    return (static_cast<DateTime>(ticks) * DATETIME_SEC) / TX_TIMER_TICKS_PER_SECOND;
}

// -- Build-timestamp epoch anchor (see clock.hpp for why the literal Unix epoch doesn't work) --
// All constexpr, evaluated once at compile time; __DATE__/__TIME__ are always "Mmm dd yyyy" /
// "hh:mm:ss" in the C/C++ standard's fixed (space-padded) format.
constexpr int MonthFromAbbrev(const char *m) {
    switch(m[0]) {
    case 'J': return m[1] == 'a' ? 1 : (m[2] == 'n' ? 6 : 7);   // Jan, Jun, Jul
    case 'F': return 2;
    case 'M': return m[2] == 'r' ? 3 : 5;                       // Mar, May
    case 'A': return m[1] == 'p' ? 4 : 8;                       // Apr, Aug
    case 'S': return 9;
    case 'O': return 10;
    case 'N': return 11;
    default:  return 12;                                        // Dec
    }
}

// Days since 1970-01-01 for a given civil (proleptic Gregorian) date -- Howard Hinnant's
// well-known constant-time algorithm (http://howardhinnant.github.io/date_algorithms.html).
constexpr int64_t DaysFromCivil(int64_t y, int m, int d) {
    y -= (m <= 2) ? 1 : 0;
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const int64_t yoe = y - era * 400;
    const int64_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

constexpr int64_t ParseBuildUnixSeconds() {
    const char *date = __DATE__; // e.g. "Aug 19 2026" (day is space-padded, e.g. "Aug  9 2026")
    const char *time = __TIME__; // "hh:mm:ss"
    int month = MonthFromAbbrev(date);
    int day = (date[4] == ' ' ? 0 : (date[4] - '0') * 10) + (date[5] - '0');
    int64_t year = (date[7] - '0') * 1000 + (date[8] - '0') * 100 + (date[9] - '0') * 10 + (date[10] - '0');
    int hour = (time[0] - '0') * 10 + (time[1] - '0');
    int minute = (time[3] - '0') * 10 + (time[4] - '0');
    int second = (time[6] - '0') * 10 + (time[7] - '0');
    return DaysFromCivil(year, month, day) * 86400 + hour * 3600 + minute * 60 + second;
}

constexpr int64_t BUILD_UNIX_SECONDS = ParseBuildUnixSeconds();

// -- SNTP-derived offset (see clock.hpp) -- guarded by g_syncMutex since ReportSntpSync() (the
// SNTP client's own internal ThreadX thread, see sntp_setup.cpp) and Now() (called from OPC UA
// connection threads) run concurrently. Stored as a 100ns-tick (DateTime) offset rather than
// whole seconds so Now()'s sub-second precision (from tx_time_get() directly) is unaffected by
// whether a sync has happened yet.
TX_MUTEX g_syncMutex;
bool g_synced = false;
DateTime g_syncOffsetHundredNs = 0; // valid only if g_synced

} // namespace

void InitClockSync() {
    tx_mutex_create(&g_syncMutex, const_cast<CHAR *>("OPC UA Clock Sync Mutex"), TX_INHERIT);
}

void ReportSntpSync(UInt32 utcSecondsSinceEpoch) {
    DateTime nowHundredNs = TicksToHundredNs(tx_time_get());
    DateTime utcHundredNs = static_cast<DateTime>(utcSecondsSinceEpoch) * DATETIME_SEC + DATETIME_UNIX_EPOCH;
    tx_mutex_get(&g_syncMutex, TX_WAIT_FOREVER);
    g_syncOffsetHundredNs = utcHundredNs - nowHundredNs;
    g_synced = true;
    tx_mutex_put(&g_syncMutex);
}

bool IsSntpSynced() {
    tx_mutex_get(&g_syncMutex, TX_WAIT_FOREVER);
    bool synced = g_synced;
    tx_mutex_put(&g_syncMutex);
    return synced;
}

DateTime Now() {
    tx_mutex_get(&g_syncMutex, TX_WAIT_FOREVER);
    bool synced = g_synced;
    DateTime offset = g_syncOffsetHundredNs;
    tx_mutex_put(&g_syncMutex);

    DateTime nowHundredNs = TicksToHundredNs(tx_time_get());
    if(synced)
        return nowHundredNs + offset;
    return nowHundredNs + DATETIME_UNIX_EPOCH + static_cast<DateTime>(BUILD_UNIX_SECONDS) * DATETIME_SEC;
}

DateTime NowMonotonic() {
    return TicksToHundredNs(tx_time_get());
}

} // namespace opcua
