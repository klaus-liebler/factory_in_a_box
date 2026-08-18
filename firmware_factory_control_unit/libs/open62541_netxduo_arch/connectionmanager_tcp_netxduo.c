/* TCP ConnectionManager built on NX_TCPSERVER (libs/ST/netxduo/addons/web/nx_tcpserver.h)
 * instead of raw NX_TCP_SOCKET/select -- reuses the same accept/session/timeout machinery
 * already proven by Core/Src/http_websocket_server.hpp. See open62541_netxduo_arch.h for the
 * threading model (one recursive TX_MUTEX, held by the EventLoop, serializes this file's
 * NX_TCPSERVER-thread callbacks against the application's dedicated "OPC UA pump" thread).
 *
 * Contract implemented here matches libs/open62541/arch/eventloop_posix_tcp.c (the reference
 * this was ported from) except: only ONE listening connection is supported (this application
 * never opens more than one "opc.tcp://" server URL -- no per-hostname fan-out), and there is
 * no active/outgoing connection support (this is a server-only port, no OPC UA client use case
 * on this board).
 *
 * connectionId 0 is reserved for the listening pseudo-connection (mirrors POSIX using the
 * listen socket's own fd as its connectionId). Real sessions use (session index + 1), where
 * the index is NX_TCPSERVER's own fixed session slot number. */
#include "open62541_netxduo_arch.h"
#include "eventloop_common.h"
#include "nx_tcpserver.h"
#include "tx_api.h"

#define UA_NX_TCP_MAX_SESSIONS 4

typedef struct {
    UA_ConnectionManager cm; /* MUST be first: open62541 core casts UA_EventSource* <-> UA_ConnectionManager* freely */
    NX_TCPSERVER tcpServer;
    NX_IP *ipPtr;
    NX_PACKET_POOL *packetPool;
    void *threadStackPtr;
    UINT threadStackSize;
    UINT threadPriority;
    UINT sessionTimeoutSeconds;

    UA_Boolean listening;
    UA_ConnectionManager_connectionCallback connectionCallback;
    void *application;
    void *listenContext;

    NX_TCP_SESSION sessions[UA_NX_TCP_MAX_SESSIONS];
    void *sessionContext[UA_NX_TCP_MAX_SESSIONS];
    UA_Boolean sessionActive[UA_NX_TCP_MAX_SESSIONS];
    UA_Boolean sessionClosePending[UA_NX_TCP_MAX_SESSIONS];
    UA_DelayedCallback closeCallbacks[UA_NX_TCP_MAX_SESSIONS];
} UA_NXConnectionManager;

static UINT
SessionIndex(UA_NXConnectionManager *ncm, NX_TCP_SESSION *session) {
    return (UINT)(session - ncm->sessions);
}

/* Actually tears a session down: notifies CLOSING, disconnects/unaccepts the socket, then
 * clears bookkeeping. Always run as a delayed callback (see RequestSessionClose below) so it
 * never executes from within a connectionCallback() call for the SAME session that is still
 * on the stack (e.g. the core deciding to close the connection synchronously while we are
 * still inside NXTCP_onReceiveData's dispatch loop for it) -- mirrors POSIX's deferred
 * TCP_delayedClose for the same reason. Runs with elMutex already held (NX_run() holds it
 * for its entire ProcessDelayed() call). */
static void
NXTCP_delayedClose(void *application, void *context) {
    UA_NXConnectionManager *ncm = (UA_NXConnectionManager*)application;
    UINT index = (UINT)(uintptr_t)context;
    NX_TCP_SESSION *session = &ncm->sessions[index];

    if(ncm->connectionCallback) {
        ncm->connectionCallback(&ncm->cm, (uintptr_t)(index + 1), ncm->application,
                                &ncm->sessionContext[index], UA_CONNECTIONSTATE_CLOSING,
                                &UA_KEYVALUEMAP_NULL, UA_BYTESTRING_NULL);
    }

    nx_tcp_socket_disconnect(&session->nx_tcp_session_socket, NX_NO_WAIT);
    nx_tcp_server_socket_unaccept(&session->nx_tcp_session_socket);

    ncm->sessionActive[index] = false;
    ncm->sessionClosePending[index] = false;
    ncm->sessionContext[index] = NULL;
}

/* Assumes elMutex is already held by the caller. */
static void
RequestSessionClose(UA_NXConnectionManager *ncm, UINT index) {
    if(!ncm->sessionActive[index] || ncm->sessionClosePending[index])
        return;
    ncm->sessionClosePending[index] = true;
    ncm->closeCallbacks[index].callback = NXTCP_delayedClose;
    ncm->closeCallbacks[index].application = ncm;
    ncm->closeCallbacks[index].context = (void*)(uintptr_t)index;
    UA_EventLoop *el = ncm->cm.eventSource.eventLoop;
    el->addDelayedCallback(el, &ncm->closeCallbacks[index]);
}

/* --- NX_TCPSERVER callbacks (run on its own worker thread) --- */

static void
NXTCP_onNewConnection(NX_TCPSERVER *server_ptr, NX_TCP_SESSION *session_ptr) {
    UA_NXConnectionManager *ncm = (UA_NXConnectionManager*)(uintptr_t)server_ptr->nx_tcpserver_reserved;
    UA_EventLoop *el = ncm->cm.eventSource.eventLoop;
    el->lock(el);

    UINT index = SessionIndex(ncm, session_ptr);
    /* Inherit whatever context the listening connection currently holds, exactly as POSIX's
     * TCP_listenSocketCallback copies conn->context into newConn->context before announcing
     * the new connection (see eventloop_posix_tcp.c). */
    ncm->sessionContext[index] = ncm->listenContext;
    ncm->sessionActive[index] = true;
    ncm->sessionClosePending[index] = false;

    ncm->connectionCallback(&ncm->cm, (uintptr_t)(index + 1), ncm->application,
                            &ncm->sessionContext[index], UA_CONNECTIONSTATE_ESTABLISHED,
                            &UA_KEYVALUEMAP_NULL, UA_BYTESTRING_NULL);

    el->unlock(el);
}

static void
NXTCP_onReceiveData(NX_TCPSERVER *server_ptr, NX_TCP_SESSION *session_ptr) {
    UA_NXConnectionManager *ncm = (UA_NXConnectionManager*)(uintptr_t)server_ptr->nx_tcpserver_reserved;
    UA_EventLoop *el = ncm->cm.eventSource.eventLoop;
    el->lock(el);

    UINT index = SessionIndex(ncm, session_ptr);
    if(!ncm->sessionActive[index] || ncm->sessionClosePending[index]) {
        el->unlock(el);
        return;
    }

    for(;;) {
        NX_PACKET *packet = NX_NULL;
        if(nx_tcp_socket_receive(&session_ptr->nx_tcp_session_socket, &packet, NX_NO_WAIT) != NX_SUCCESS)
            break;

        /* Deliver each chained NX_PACKET fragment as its own connectionCallback invocation --
         * TCP is a byte stream, open62541's binary chunk reassembly (ua_securechannel.c)
         * already handles arbitrary read/write boundaries, no need to coalesce first. */
        for(NX_PACKET *p = packet; p != NX_NULL; p = p->nx_packet_next) {
            if(ncm->sessionClosePending[index])
                break;
            UA_ByteString msg;
            msg.data = p->nx_packet_prepend_ptr;
            msg.length = (size_t)(p->nx_packet_append_ptr - p->nx_packet_prepend_ptr);
            ncm->connectionCallback(&ncm->cm, (uintptr_t)(index + 1), ncm->application,
                                    &ncm->sessionContext[index], UA_CONNECTIONSTATE_ESTABLISHED,
                                    &UA_KEYVALUEMAP_NULL, msg);
        }
        nx_packet_release(packet);

        if(ncm->sessionClosePending[index])
            break;
    }

    el->unlock(el);
}

static void
NXTCP_onConnectionEnd(NX_TCPSERVER *server_ptr, NX_TCP_SESSION *session_ptr) {
    UA_NXConnectionManager *ncm = (UA_NXConnectionManager*)(uintptr_t)server_ptr->nx_tcpserver_reserved;
    UA_EventLoop *el = ncm->cm.eventSource.eventLoop;
    el->lock(el);
    RequestSessionClose(ncm, SessionIndex(ncm, session_ptr));
    el->unlock(el);
}

static void
NXTCP_onConnectionTimeout(NX_TCPSERVER *server_ptr, NX_TCP_SESSION *session_ptr) {
    /* Same handling as a remote-initiated close (idle-timeout is just another reason the
     * session needs to go away) -- nx_tcpserver already re-listens the slot internally for
     * the timeout case (_nx_tcpserver_timeout_process), our unaccept()/disconnect() in
     * NXTCP_delayedClose() is still required and idempotent either way. */
    NXTCP_onConnectionEnd(server_ptr, session_ptr);
}

/* --- UA_ConnectionManager interface --- */

static UA_StatusCode
NXTCP_openConnection(UA_ConnectionManager *cm, const UA_KeyValueMap *params,
                     void *application, void *context,
                     UA_ConnectionManager_connectionCallback connectionCallback) {
    UA_NXConnectionManager *ncm = (UA_NXConnectionManager*)cm;
    UA_EventLoop *el = cm->eventSource.eventLoop;

    if(cm->eventSource.state != UA_EVENTSOURCESTATE_STARTED)
        return UA_STATUSCODE_BADINTERNALERROR;

    UA_Boolean listen = false;
    const UA_Boolean *listenParam = (const UA_Boolean*)
        UA_KeyValueMap_getScalar(params, UA_QUALIFIEDNAME(0, (char*)(uintptr_t)"listen"),
                                 &UA_TYPES[UA_TYPES_BOOLEAN]);
    if(listenParam)
        listen = *listenParam;
    if(!listen) {
        UA_LOG_ERROR(el->logger, UA_LOGCATEGORY_NETWORK,
                     "NXTCP\t| Only listening connections are supported (server-only port)");
        return UA_STATUSCODE_BADNOTSUPPORTED;
    }

    if(ncm->listening) {
        UA_LOG_ERROR(el->logger, UA_LOGCATEGORY_NETWORK,
                     "NXTCP\t| Already listening -- only one listener is supported");
        return UA_STATUSCODE_BADALREADYEXISTS;
    }

    const UA_UInt16 *portParam = (const UA_UInt16*)
        UA_KeyValueMap_getScalar(params, UA_QUALIFIEDNAME(0, (char*)(uintptr_t)"port"),
                                 &UA_TYPES[UA_TYPES_UINT16]);
    UA_UInt16 port = portParam ? *portParam : 4840;

    el->lock(el);

    UINT status = nx_tcpserver_create(
        ncm->ipPtr, &ncm->tcpServer, (CHAR*)(uintptr_t)"OPC UA TCP Server",
        NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 8192,
        NXTCP_onNewConnection, NXTCP_onReceiveData,
        NXTCP_onConnectionEnd, NXTCP_onConnectionTimeout,
        ncm->sessionTimeoutSeconds, ncm->threadStackPtr, ncm->threadStackSize,
        ncm->sessions, sizeof(ncm->sessions), ncm->threadPriority, NX_IP_PERIODIC_RATE);
    if(status != NX_SUCCESS) {
        UA_LOG_ERROR(el->logger, UA_LOGCATEGORY_NETWORK,
                     "NXTCP\t| nx_tcpserver_create failed with status 0x%x", status);
        el->unlock(el);
        return UA_STATUSCODE_BADINTERNALERROR;
    }
    /* Back-pointer for the NX_TCPSERVER callbacks above to recover "ncm" from the bare
     * NX_TCPSERVER* they are handed -- nx_tcpserver_reserved exists in the struct
     * specifically for application use (see nx_tcpserver.h). */
    ncm->tcpServer.nx_tcpserver_reserved = (ULONG)(uintptr_t)ncm;

    status = nx_tcpserver_start(&ncm->tcpServer, port, UA_NX_TCP_MAX_SESSIONS * 2);
    if(status != NX_SUCCESS) {
        UA_LOG_ERROR(el->logger, UA_LOGCATEGORY_NETWORK,
                     "NXTCP\t| nx_tcpserver_start failed with status 0x%x", status);
        nx_tcpserver_delete(&ncm->tcpServer);
        el->unlock(el);
        return UA_STATUSCODE_BADINTERNALERROR;
    }

    ncm->connectionCallback = connectionCallback;
    ncm->application = application;
    ncm->listenContext = context;
    ncm->listening = true;

    UA_String listenAddress = UA_STRING((char*)(uintptr_t)"0.0.0.0");
    UA_KeyValuePair listenParams[2];
    listenParams[0].key = UA_QUALIFIEDNAME(0, (char*)(uintptr_t)"listen-address");
    UA_Variant_setScalar(&listenParams[0].value, &listenAddress, &UA_TYPES[UA_TYPES_STRING]);
    listenParams[1].key = UA_QUALIFIEDNAME(0, (char*)(uintptr_t)"listen-port");
    UA_Variant_setScalar(&listenParams[1].value, &port, &UA_TYPES[UA_TYPES_UINT16]);
    UA_KeyValueMap listenMap = {2, listenParams};

    connectionCallback(cm, 0, application, &ncm->listenContext,
                       UA_CONNECTIONSTATE_ESTABLISHED, &listenMap, UA_BYTESTRING_NULL);

    el->unlock(el);
    return UA_STATUSCODE_GOOD;
}

static UA_StatusCode
NXTCP_closeConnection(UA_ConnectionManager *cm, uintptr_t connectionId) {
    UA_NXConnectionManager *ncm = (UA_NXConnectionManager*)cm;
    UA_EventLoop *el = cm->eventSource.eventLoop;
    el->lock(el);

    if(connectionId == 0 || connectionId > UA_NX_TCP_MAX_SESSIONS) {
        el->unlock(el);
        return UA_STATUSCODE_BADNOTFOUND;
    }
    UINT index = (UINT)(connectionId - 1);
    if(!ncm->sessionActive[index] || ncm->sessionClosePending[index]) {
        el->unlock(el);
        return UA_STATUSCODE_BADNOTFOUND;
    }

    RequestSessionClose(ncm, index);

    el->unlock(el);
    return UA_STATUSCODE_GOOD;
}

static UA_StatusCode
NXTCP_sendWithConnection(UA_ConnectionManager *cm, uintptr_t connectionId,
                         const UA_KeyValueMap *params, UA_ByteString *buf) {
    (void)params;
    UA_NXConnectionManager *ncm = (UA_NXConnectionManager*)cm;

    if(connectionId == 0 || connectionId > UA_NX_TCP_MAX_SESSIONS) {
        UA_ByteString_clear(buf);
        return UA_STATUSCODE_BADCONNECTIONCLOSED;
    }
    UINT index = (UINT)(connectionId - 1);
    if(!ncm->sessionActive[index] || ncm->sessionClosePending[index]) {
        UA_ByteString_clear(buf);
        return UA_STATUSCODE_BADCONNECTIONCLOSED;
    }

    NX_PACKET *packet = NX_NULL;
    if(nx_packet_allocate(ncm->packetPool, &packet, NX_TCP_PACKET, NX_WAIT_FOREVER) != NX_SUCCESS) {
        UA_ByteString_clear(buf);
        return UA_STATUSCODE_BADOUTOFMEMORY;
    }
    if(nx_packet_data_append(packet, buf->data, (ULONG)buf->length,
                             ncm->packetPool, NX_WAIT_FOREVER) != NX_SUCCESS) {
        nx_packet_release(packet);
        UA_ByteString_clear(buf);
        return UA_STATUSCODE_BADINTERNALERROR;
    }

    NX_TCP_SESSION *session = &ncm->sessions[index];
    UINT status = nx_tcp_socket_send(&session->nx_tcp_session_socket, packet, NX_WAIT_FOREVER);
    UA_ByteString_clear(buf);
    if(status != NX_SUCCESS) {
        nx_packet_release(packet);
        UA_EventLoop *el = cm->eventSource.eventLoop;
        el->lock(el);
        RequestSessionClose(ncm, index);
        el->unlock(el);
        return UA_STATUSCODE_BADCONNECTIONCLOSED;
    }
    return UA_STATUSCODE_GOOD;
}

static UA_StatusCode
NXTCP_allocNetworkBuffer(UA_ConnectionManager *cm, uintptr_t connectionId,
                         UA_ByteString *buf, size_t bufSize) {
    (void)cm; (void)connectionId;
    return UA_ByteString_allocBuffer(buf, bufSize);
}

static void
NXTCP_freeNetworkBuffer(UA_ConnectionManager *cm, uintptr_t connectionId, UA_ByteString *buf) {
    (void)cm; (void)connectionId;
    UA_ByteString_clear(buf);
}

static UA_StatusCode
NXTCP_eventSourceStart(UA_ConnectionManager *cm) {
    if(cm->eventSource.state != UA_EVENTSOURCESTATE_STOPPED)
        return UA_STATUSCODE_BADINTERNALERROR;
    cm->eventSource.state = UA_EVENTSOURCESTATE_STARTED;
    return UA_STATUSCODE_GOOD;
}

static void
NXTCP_eventSourceStop(UA_ConnectionManager *cm) {
    UA_NXConnectionManager *ncm = (UA_NXConnectionManager*)cm;
    cm->eventSource.state = UA_EVENTSOURCESTATE_STOPPING;
    /* Embedded server: stopping only ever happens on application shutdown, not a case that
     * needs graceful per-session draining/notification -- just tear the listener down. */
    if(ncm->listening) {
        nx_tcpserver_stop(&ncm->tcpServer);
        nx_tcpserver_delete(&ncm->tcpServer);
        ncm->listening = false;
    }
    cm->eventSource.state = UA_EVENTSOURCESTATE_STOPPED;
}

static UA_StatusCode
NXTCP_eventSourceDelete(UA_ConnectionManager *cm) {
    if(cm->eventSource.state >= UA_EVENTSOURCESTATE_STARTING)
        return UA_STATUSCODE_BADINTERNALERROR;
    UA_KeyValueMap_clear(&cm->eventSource.params);
    UA_String_clear(&cm->eventSource.name);
    UA_free(cm);
    return UA_STATUSCODE_GOOD;
}

UA_ConnectionManager *
UA_ConnectionManager_new_NetXDuo_TCP(const UA_String eventSourceName, NX_IP *ipPtr,
                                     NX_PACKET_POOL *packetPool,
                                     void *threadStackPtr, UINT threadStackSize,
                                     UINT threadPriority, UINT sessionTimeoutSeconds) {
    UA_NXConnectionManager *ncm = (UA_NXConnectionManager*)UA_calloc(1, sizeof(UA_NXConnectionManager));
    if(!ncm)
        return NULL;

    ncm->cm.eventSource.eventSourceType = UA_EVENTSOURCETYPE_CONNECTIONMANAGER;
    UA_String_copy(&eventSourceName, &ncm->cm.eventSource.name);
    ncm->cm.eventSource.start = (UA_StatusCode (*)(UA_EventSource*))NXTCP_eventSourceStart;
    ncm->cm.eventSource.stop = (void (*)(UA_EventSource*))NXTCP_eventSourceStop;
    ncm->cm.eventSource.free = (UA_StatusCode (*)(UA_EventSource*))NXTCP_eventSourceDelete;
    ncm->cm.protocol = UA_STRING((char*)(uintptr_t)"tcp");
    ncm->cm.openConnection = NXTCP_openConnection;
    ncm->cm.sendWithConnection = NXTCP_sendWithConnection;
    ncm->cm.closeConnection = NXTCP_closeConnection;
    ncm->cm.allocNetworkBuffer = NXTCP_allocNetworkBuffer;
    ncm->cm.freeNetworkBuffer = NXTCP_freeNetworkBuffer;

    ncm->ipPtr = ipPtr;
    ncm->packetPool = packetPool;
    ncm->threadStackPtr = threadStackPtr;
    ncm->threadStackSize = threadStackSize;
    ncm->threadPriority = threadPriority;
    ncm->sessionTimeoutSeconds = sessionTimeoutSeconds;

    return &ncm->cm;
}
