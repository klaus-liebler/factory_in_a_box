#pragma once
// This board has no RTC with a usable clock source (LSE crystal pins are already wired to other
// peripherals, see project memory) -- real wall-clock time instead comes from SNTP once the
// network is up (Core/Src/sntp_setup.cpp, firmware_factory_control_unit's own application code,
// which calls ReportSntpSync() below on every successful sync). Before the first sync (or if the
// network never comes up), Now() falls back to anchoring tx_time_get()==0 (boot) onto the
// firmware's __DATE__/__TIME__ build timestamp -- NOT the literal Unix epoch: a real OPC UA
// client (UAExpert confirmed, 2026-08-19) computes SecureChannel token expiry as the
// server-reported createdAt plus the revised lifetime, compared against the CLIENT's own real
// UTC clock. Anchor 56 years in the past (the literal epoch) and even a 1-hour lifetime reads as
// expired the instant the channel opens, so the client perpetually thinks it needs to renew -- a
// renew/reconnect loop that looks like a connection-stability bug but is really just a wrong
// clock. The build-timestamp anchor is close enough to avoid that even pre-sync, for a dev board
// that boots shortly after being flashed.
#include "opcua/types.hpp"

namespace opcua {

// Creates the mutex ReportSntpSync()/Now() share to guard the sync offset below -- call ONCE,
// early, before any thread that might call either (mirrors this project's existing pattern of
// creating all mutexes up front in tx_application_define(), see Core/Src/app.cc).
void InitClockSync();

DateTime Now();
DateTime NowMonotonic();

// Called by sntp_setup.cpp on every successful SNTP sync -- utcSecondsSinceEpoch is real UTC
// time (Unix epoch seconds) as of THIS call (i.e. already NTP-epoch-to-Unix-epoch converted by
// the caller). Thread-safe: the SNTP client's own internal thread calls this from a different
// thread than whatever calls Now().
void ReportSntpSync(UInt32 utcSecondsSinceEpoch);
// True once at least one SNTP sync has succeeded (i.e. Now() is real UTC, not the build-
// timestamp fallback). Informational only (e.g. for a boot-log line); Now() itself always
// returns a value either way.
bool IsSntpSynced();

} // namespace opcua
