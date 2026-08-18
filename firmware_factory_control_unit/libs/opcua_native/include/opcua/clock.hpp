#pragma once
// This board has no RTC and no SNTP client -- there is no wall-clock time source at all (same
// situation documented for the open62541 port on the sibling branch). SecurityPolicy#None
// never validates certificate time, so an arbitrary epoch offset is fine: tx_time_get()==0
// (boot) maps to the Unix epoch, then re-based onto the OPC UA/Windows FILETIME epoch
// (DATETIME_UNIX_EPOCH, see types.hpp). Monotonic and correctly-paced, just not calendar-accurate.
#include "opcua/types.hpp"

namespace opcua {

DateTime Now();
DateTime NowMonotonic();

} // namespace opcua
