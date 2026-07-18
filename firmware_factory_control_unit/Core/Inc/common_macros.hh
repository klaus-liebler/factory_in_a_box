#pragma once


// printf-Format samt passender Argumentliste fuer eine NetX-ULONG-IPv4-Adresse
// (Network-Byte-Order-Oktette aus dem 32-Bit-Wert maskiert/geshiftet), z.B.:
//   log_info("IP Address: " IP_ADDR_FMT, IP_ADDR_FMT_ARGS(g_app_state.ip_address));
#define IP_ADDR_FMT "%lu.%lu.%lu.%lu"
#define IP_ADDR_FMT_ARGS(addr) \
    (((addr) >> 24) & 0xff), (((addr) >> 16) & 0xff), (((addr) >> 8) & 0xff), ((addr) & 0xff)

#define _C(str) const_cast<CHAR *>(str)

// Success-Makro fuer NX_SUCCESS-liefernde Aufrufe -- loggt Kontext (Datei/Zeile/Statuscode) und
// haelt in Error_Handler() an. Von allen Modulen gemeinsam genutzt.
#define XASSERT(status_expr, message_on_fail) \
do { \
    UINT assure_status_ = (status_expr); \
    if (assure_status_ != NX_SUCCESS) { \
        log_error("Error %s: %s:%d, status: 0x%x", message_on_fail, __FILE__, __LINE__, assure_status_); \
        Error_Handler(); \
    } \
} while (0)
