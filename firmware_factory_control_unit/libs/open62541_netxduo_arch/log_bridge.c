/* Bridges open62541's UA_Logger plugin interface to this project's own log.h (log_info()/
 * log_warn()/... over UART + WebSocket mirror, see stm32_libs/common_stm32/log.h) instead of
 * open62541's default UA_Log_Stdout (this board never retargets stdio/printf to anything
 * useful). */
#include <open62541/plugin/log.h>
#include <stdio.h>
#include "log.h"

static const char *
CategoryName(UA_LogCategory category) {
    static const char *names[UA_LOGCATEGORIES] = {
        "network", "channel", "session", "server", "client",
        "userland", "securitypolicy", "eventloop", "pubsub", "discovery"
    };
    return ((unsigned)category < UA_LOGCATEGORIES) ? names[category] : "?";
}

static int
ProjectLevel(UA_LogLevel level) {
    if(level >= UA_LOGLEVEL_FATAL) return LOG_FATAL;
    if(level >= UA_LOGLEVEL_ERROR) return LOG_ERROR;
    if(level >= UA_LOGLEVEL_WARNING) return LOG_WARN;
    if(level >= UA_LOGLEVEL_INFO) return LOG_INFO;
    if(level >= UA_LOGLEVEL_DEBUG) return LOG_DEBUG;
    return LOG_TRACE;
}

static void
OpcUaLog(void *context, UA_LogLevel level, UA_LogCategory category,
        const char *msg, va_list args) {
    (void)context;
    char buf[192];
    vsnprintf(buf, sizeof(buf), msg, args);
    log_log(ProjectLevel(level), "opcua", 0, "[%s] %s", CategoryName(category), buf);
}

/* Not const: UA_ServerConfig.logging is typed "UA_Logger *" upstream (matches what
 * UA_Log_Stdout_new() returns -- a heap-allocated, mutable logger). This instance is
 * static/immutable in practice (its .clear is NULL, so open62541 never calls anything on it
 * that would mutate state), but is exposed with the same non-const type for drop-in
 * compatibility with config.logging's declared type. */
static UA_Logger g_opcUaLogger = {OpcUaLog, NULL, NULL};

UA_Logger *
OpcUa_Log_Project(void) {
    return &g_opcUaLogger;
}
