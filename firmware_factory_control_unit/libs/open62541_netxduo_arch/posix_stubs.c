/* setDefaultConfig() (libs/open62541/plugins/ua_config_default.c, called by every
 * UA_ServerConfig_setDefault()/_setBasics()/_setMinimalCustomBuffer()/... entry point)
 * unconditionally calls UA_EventLoop_new_POSIX()/UA_ConnectionManager_new_POSIX_TCP()/_UDP()/
 * UA_InterruptManager_new_POSIX() -- but only inside "if(conf->eventLoop == NULL)". This
 * project always pre-populates config.eventLoop with UA_EventLoop_new_NetXDuo() +
 * UA_ConnectionManager_new_NetXDuo_TCP() (see opcua_setup.cpp) BEFORE calling any
 * UA_ServerConfig_set*() function, so that branch never actually executes at runtime.
 *
 * However it's a runtime "if", not "#ifdef": the calls are still compiled into
 * setDefaultConfig(), so the four symbols below must exist for the link to succeed even
 * though they are provably unreachable here. Trivial stubs. */
#include <open62541/plugin/eventloop.h>

UA_EventLoop *
UA_EventLoop_new_POSIX(const UA_Logger *logger) {
    (void)logger;
    return NULL;
}

UA_ConnectionManager *
UA_ConnectionManager_new_POSIX_TCP(const UA_String eventSourceName) {
    (void)eventSourceName;
    return NULL;
}

UA_ConnectionManager *
UA_ConnectionManager_new_POSIX_UDP(const UA_String eventSourceName) {
    (void)eventSourceName;
    return NULL;
}

UA_InterruptManager *
UA_InterruptManager_new_POSIX(const UA_String eventSourceName) {
    (void)eventSourceName;
    return NULL;
}
