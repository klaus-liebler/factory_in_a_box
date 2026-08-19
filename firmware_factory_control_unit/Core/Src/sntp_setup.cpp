#include "sntp_setup.hpp"

#include "app.hh"
#include "common_macros.hh"
#include "log.h"
#include "opcua/clock.hpp"

extern "C" {
#include "nx_api.h"
#include "nxd_dns.h"
#include "nxd_sntp_client.h"
}

namespace {

// Interface 0 is Ethernet (the one with an actual route to the internet); interface 1 is
// USB-NCM, a point-to-point link to the host PC only (see net_setup.cpp's
// USB_NCM_INTERFACE_INDEX) -- SNTP has nothing to reach on that one.
constexpr UINT SNTP_INTERFACE_INDEX = 0;
constexpr const char *NTP_SERVER_HOSTNAME = "pool.ntp.org";
// No DHCP-provided DNS server is read out here (would need
// nx_dhcp_interface_user_option_retrieve with the DNS suboption -- not implemented, deliberately
// out of scope for now); a well-known public resolver works regardless of what the local router
// hands out.
constexpr ULONG DNS_SERVER_ADDRESS = IP_ADDRESS(8, 8, 8, 8);
// NTP's epoch is 1900-01-01, OPC UA/Unix's is 1970-01-01 -- 70 years, the standard, widely-used
// constant (matches e.g. ST's own Nx_SNTP_Client sample's EPOCH_TIME_DIFF).
constexpr ULONG NTP_TO_UNIX_EPOCH_SECONDS = 2208988800UL;
constexpr UINT SNTP_THREAD_STACK_SIZE = 6 * 1024; // matches ST's own Nx_SNTP_Client sample
// Same band as the IO thread (app.cc) -- reacts to network/timer events, not latency-critical.
constexpr UINT SNTP_THREAD_PRIORITY = 8;
constexpr ULONG NETWORK_READY_POLL_TICKS = 5 * TX_TIMER_TICKS_PER_SECOND;
constexpr ULONG DNS_LOOKUP_TIMEOUT_TICKS = 10 * TX_TIMER_TICKS_PER_SECOND;

TX_THREAD g_sntpThread;
NX_DNS g_dnsClient;
NX_SNTP_CLIENT g_sntpClient;
App *g_app = nullptr;

// Codes that mean "try again later" get a chance to; anything else (auth-related codes this
// server never uses, or an unrecognized code) removes the server from the client's active list,
// matching ST's own sample's handling -- this server has no fallback NTP server list, so a
// removed server just means no further updates until the client is recreated (not implemented;
// this board only ever configures one NTP server).
UINT KissOfDeathHandler(NX_SNTP_CLIENT * /*clientPtr*/, UINT kodCode) {
    switch(kodCode) {
    case NX_SNTP_KOD_RATE:
    case NX_SNTP_KOD_NOT_INIT:
    case NX_SNTP_KOD_STEP:
        return NX_SNTP_KOD_SERVER_NOT_AVAILABLE;
    default:
        return NX_SNTP_KOD_REMOVE_SERVER;
    }
}

// Called from the SNTP client's own internal ThreadX thread (started by nx_sntp_client_run_
// unicast(), keeps re-syncing periodically on its own -- see nxd_sntp_client.c's doc comments)
// on every valid update it receives, for as long as this program runs.
void TimeUpdateNotify(NX_SNTP_TIME_MESSAGE * /*messagePtr*/, NX_SNTP_TIME *localTime) {
    uint32_t unixSeconds = static_cast<uint32_t>(localTime->seconds - NTP_TO_UNIX_EPOCH_SECONDS);
    opcua::ReportSntpSync(unixSeconds);
    log_info("SNTP: synced, UTC epoch seconds=%lu", (unsigned long)unixSeconds);
}

void SntpThreadEntry(ULONG /*threadInput*/) {
    UINT status;

    // Wait for the Ethernet interface to actually have a real DHCP-leased IP -- NOT
    // nx_ip_interface_status_check(..., NX_IP_ADDRESS_RESOLVED, ...): that's an ARP conflict
    // check for whatever address is CURRENTLY configured, and net_setup.cpp's nx_ip_create()
    // starts interface 0 at the literal address 0 (NX_APP_DEFAULT_IP_ADDRESS) before DHCP runs
    // -- there's nothing to ARP-conflict-check for 0.0.0.0, so that status is satisfied almost
    // immediately, well before DHCP actually completes (confirmed 2026-08-19: caused every SNTP
    // DNS lookup to fail with NX_DNS_QUERY_FAILED, sent from a still-unconfigured interface).
    // Poll for a genuinely non-zero address instead. Retries forever at a slow poll rate instead
    // of giving up: the network may come up long after boot (cable plugged in later, DHCP server
    // briefly unavailable, ...). clock.cpp's build-timestamp fallback covers Now() meanwhile.
    for(;;) {
        ULONG ipAddress = 0;
        ULONG netMask = 0;
        status = nx_ip_interface_address_get(&g_app->ip_instance, SNTP_INTERFACE_INDEX, &ipAddress, &netMask);
        if(status == NX_SUCCESS && ipAddress != 0)
            break;
        tx_thread_sleep(NETWORK_READY_POLL_TICKS);
    }

    if(nx_dns_create(&g_dnsClient, &g_app->ip_instance, reinterpret_cast<UCHAR *>(_C("DNS Client"))) != NX_SUCCESS) {
        log_error("SNTP: DNS client create failed");
        return;
    }
    if(nx_dns_server_add(&g_dnsClient, DNS_SERVER_ADDRESS) != NX_SUCCESS) {
        log_error("SNTP: DNS server add failed");
        return;
    }

    ULONG ntpServerAddress = 0;
    status = nx_dns_host_by_name_get(&g_dnsClient, reinterpret_cast<UCHAR *>(_C(NTP_SERVER_HOSTNAME)),
                                     &ntpServerAddress, DNS_LOOKUP_TIMEOUT_TICKS);
    if(status != NX_SUCCESS) {
        log_error("SNTP: DNS lookup of %s failed (status=0x%x)", NTP_SERVER_HOSTNAME, status);
        return;
    }
    log_info("SNTP: %s resolved to " IP_ADDR_FMT, NTP_SERVER_HOSTNAME, IP_ADDR_FMT_ARGS(ntpServerAddress));

    if(nx_sntp_client_create(&g_sntpClient, &g_app->ip_instance, SNTP_INTERFACE_INDEX, &g_app->packet_pool,
                             nullptr, KissOfDeathHandler, nullptr) != NX_SUCCESS) {
        log_error("SNTP: client create failed");
        return;
    }
    nx_sntp_client_set_time_update_notify(&g_sntpClient, TimeUpdateNotify);

    if(nx_sntp_client_initialize_unicast(&g_sntpClient, ntpServerAddress) != NX_SUCCESS) {
        log_error("SNTP: initialize_unicast failed");
        return;
    }
    if(nx_sntp_client_run_unicast(&g_sntpClient) != NX_SUCCESS) {
        log_error("SNTP: run_unicast failed");
        return;
    }
    log_info("SNTP: client started against %s", NTP_SERVER_HOSTNAME);
    // From here on the SNTP client's own internal thread/timer (started by run_unicast above)
    // keeps re-syncing periodically and calls TimeUpdateNotify() on every update -- this thread
    // has nothing further to do and can just end (its stack stays allocated, matching how
    // opcua_setup.hpp's setup function also returns once its own worker thread is spawned).
}

} // namespace

void SntpSetup(App *app) {
    g_app = app;
    void *stackPtr = nullptr;
    XASSERT(tx_byte_allocate(&app->byte_pool, &stackPtr, SNTP_THREAD_STACK_SIZE, TX_NO_WAIT),
            "SNTP thread stack allocate failed");
    XASSERT(tx_thread_create(&g_sntpThread, _C("SNTP"), SntpThreadEntry, 0, stackPtr,
                             SNTP_THREAD_STACK_SIZE, SNTP_THREAD_PRIORITY, SNTP_THREAD_PRIORITY,
                             TX_NO_TIME_SLICE, TX_AUTO_START),
            "SNTP thread create failed");
}
