#!/usr/bin/env python3
"""HTTPS + WebSocket smoke test for the Control Unit's Http::WebServer
(Core/Src/http_websocket_server.hpp, wired up in Core/Src/webserver.cpp).

Exercises exactly the two things the recent RAM/network rework touched:
  1. HTTPS (NetX Secure TLS) now uses the SAME RSA-2048 device certificate the OPC UA server
     already used (see net_setup.cpp/opcua_setup.cpp -- a separate EC_P256 certificate for HTTPS
     was tried and then reverted, see the project's certificate-consolidation notes) -- a plain
     TLS GET of the SPA shell ("/") is enough to prove the RSA cert/TLS handshake still works on
     this no-PKA board (STM32H563) in a Release build.
  2. The WebSocket endpoint ("/ws") speaks the BestBinaryBuffer wire format (best_binary_buffers_
     schema/*.cs -> Core/generated/ws_protocol.hh / web/generated/ws-protocol.ts) -- this test
     sends a real tasks.TaskManagerRequest and decodes the tasks.TaskListMessage response
     (thread name/state/stack usage/CPU load per ThreadX thread), the same round trip the
     Task Manager web UI page performs.

Needs 'cryptography' (reading the CA cert, same dependency as test_opcua.py) and 'websockets'
(RFC6455 client) -- see tools/requirements.txt (pip install -r tools/requirements.txt).

Trust model: like test_opcua.py, this server's certificate must be trusted out of band -- pass
the private CA's certificate via --ca-cert (e.g. the "rootCA.pem.crt" configured as
Certificates:CertsDir in builder/appsettings.json) so normal chain+hostname verification runs
against the board's real SAN entries (hostname, mDNS ".local" name, IP). Passing the leaf
certificate itself also works (a certificate is always trusted to vouch for itself).

Examples:
    python test-https-and-websocket.py --host factory-box-363836.local --ca-cert C:\\path\\to\\rootCA.pem.crt
    python test-https-and-websocket.py --host 192.168.1.50 --ca-cert rootCA.pem.crt
"""
import argparse
import http.client
import ssl
import struct
import sys

import websockets.sync.client as ws_client

HTTPS_PORT = 443
WS_PATH = "/ws"

# Wire ids from best_binary_buffers_schema/tasks.cs, as persisted in best_binary_buffers_schema/
# ids.txt and emitted into Core/generated/ws_protocol.hh -- kept in sync by hand here (like
# test_opcua.py's NS0_* NodeIds), this is a test tool, not generated code. Re-check
# Core/generated/ws_protocol.hh's "namespace tasks { constexpr uint16_t NAMESPACE_ID = ...}" /
# "TaskManagerRequest::TYPE_ID" / "TaskListMessage::TYPE_ID" if these ever look wrong.
TASKS_NAMESPACE_ID = 3
TASK_MANAGER_REQUEST_TYPE_ID = 1
TASK_LIST_MESSAGE_TYPE_ID = 2
TASK_INFO_ELEMENT_SIZE = 52  # name[32] + state(1) + priority(1) + 4x uint32 + uint16, see TaskInfo_SIZE

THREAD_STATE_NAMES = {
    0: "Ready", 1: "Completed", 2: "Terminated", 3: "Suspended", 4: "Sleep",
    5: "QueueSuspended", 6: "SemaphoreSuspended", 7: "EventFlag", 8: "BlockMemory",
    9: "ByteMemory", 10: "IoDriver", 11: "File", 12: "TcpIp", 13: "MutexSuspended",
    14: "PriorityChange", 255: "Unknown",
}


def build_ssl_context(ca_cert_path: str) -> ssl.SSLContext:
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.load_verify_locations(cafile=ca_cert_path)
    return ctx


def test_https(host: str, port: int, ssl_ctx: ssl.SSLContext) -> None:
    print(f"HTTPS GET https://{host}:{port}/ ...")
    conn = http.client.HTTPSConnection(host, port, timeout=10.0, context=ssl_ctx)
    try:
        conn.request("GET", "/")
        resp = conn.getresponse()
        body = resp.read()
        print(f"  status={resp.status} {resp.reason}")
        content_encoding = resp.getheader("Content-Encoding")
        content_type = resp.getheader("Content-Type")
        print(f"  Content-Type={content_type!r} Content-Encoding={content_encoding!r} bytes={len(body)}")
        if resp.status != 200:
            print(f"FAIL: expected HTTP 200 for GET /, got {resp.status}")
            sys.exit(1)
        # SPA shell is always served brotli-compressed (handle_spa_shell() in webserver.cpp,
        # streamed straight from the embedded index.html.br flash asset) -- not re-checking the
        # compressed body's content here, just that the TLS handshake + a real HTTP response made
        # it through with the expected framing.
        if content_encoding != "br":
            print(f"FAIL: expected Content-Encoding: br, got {content_encoding!r}")
            sys.exit(1)
        if len(body) == 0:
            print("FAIL: empty response body for GET /")
            sys.exit(1)
    finally:
        conn.close()
    print("  HTTPS OK (TLS handshake + RSA certificate verified, SPA shell served)")


def _encode_task_manager_request(request_id: int) -> bytes:
    # [namespaceId u16 LE][typeId u16 LE][requestId u16 LE] -- TaskManagerRequest has no further
    # fields (see best_binary_buffers_schema/tasks.cs), matches TaskManagerRequest_MAX_SIZE = 6.
    return struct.pack("<HHH", TASKS_NAMESPACE_ID, TASK_MANAGER_REQUEST_TYPE_ID, request_id)


def _decode_task_list_message(data: bytes, expected_request_id: int) -> list[dict]:
    if len(data) < 4:
        raise ValueError(f"frame too short for a header: {len(data)} bytes")
    namespace_id, type_id = struct.unpack_from("<HH", data, 0)
    if namespace_id != TASKS_NAMESPACE_ID or type_id != TASK_LIST_MESSAGE_TYPE_ID:
        raise ValueError(f"unexpected frame namespaceId={namespace_id} typeId={type_id} "
                          f"(expected tasks.TaskListMessage = {TASKS_NAMESPACE_ID}/{TASK_LIST_MESSAGE_TYPE_ID})")
    pos = 4
    if len(data) < pos + 2:
        raise ValueError("frame too short for requestId")
    request_id, = struct.unpack_from("<H", data, pos)
    pos += 2
    if request_id != expected_request_id:
        raise ValueError(f"requestId mismatch: sent {expected_request_id}, got {request_id} back")
    if len(data) < pos + 2:
        raise ValueError("frame too short for items count prefix")
    count, = struct.unpack_from("<H", data, pos)
    pos += 2
    needed = count * TASK_INFO_ELEMENT_SIZE
    if len(data) < pos + needed:
        raise ValueError(f"frame too short for {count} TaskInfo elements ({needed} bytes needed, "
                          f"{len(data) - pos} available)")
    tasks = []
    for i in range(count):
        base = pos + i * TASK_INFO_ELEMENT_SIZE
        name_raw = data[base:base + 32]
        name = name_raw.split(b"\x00", 1)[0].decode("ascii", errors="replace")
        state, priority, stack_size, stack_used_current, stack_used_high_water, run_count, cpu_permille = \
            struct.unpack_from("<BBIIIIH", data, base + 32)
        tasks.append({
            "name": name,
            "state": THREAD_STATE_NAMES.get(state, f"?({state})"),
            "priority": priority,
            "stackSizeBytes": stack_size,
            "stackUsedCurrentBytes": stack_used_current,
            "stackUsedHighWaterBytes": stack_used_high_water,
            "runCount": run_count,
            "cpuPercent": cpu_permille / 10.0,
        })
    return tasks


def test_websocket(host: str, port: int, ssl_ctx: ssl.SSLContext) -> None:
    uri = f"wss://{host}:{port}{WS_PATH}"
    print(f"WebSocket connecting to {uri} ...")
    with ws_client.connect(uri, ssl_context=ssl_ctx, open_timeout=10.0) as ws:
        print("  connected, sending tasks.TaskManagerRequest ...")
        request_id = 42
        ws.send(_encode_task_manager_request(request_id))
        # The server may interleave OTHER traffic on this same connection at any time -- notably
        # system.LogMessage (WsLogBridge mirrors every firmware log line to all connected WS
        # clients, unrelated to this request/response). A real client dispatches by
        # namespaceId/typeId and ignores what it doesn't recognize (see ws-client.ts's
        # handleMessage()); this test does the same instead of assuming the first frame back is
        # necessarily the answer.
        tasks = None
        for _ in range(20):
            frame = ws.recv(timeout=10.0)
            if isinstance(frame, str):
                print(f"  (ignoring unexpected text frame: {frame!r})")
                continue
            if len(frame) >= 4 and struct.unpack_from("<HH", frame, 0) == (TASKS_NAMESPACE_ID, TASK_LIST_MESSAGE_TYPE_ID):
                tasks = _decode_task_list_message(frame, request_id)
                break
            ns_id, ty_id = struct.unpack_from("<HH", frame, 0) if len(frame) >= 4 else (None, None)
            print(f"  (ignoring unrelated binary frame: namespaceId={ns_id} typeId={ty_id}, {len(frame)} bytes)")
        if tasks is None:
            print("FAIL: did not receive a tasks.TaskListMessage response within 20 frames")
            sys.exit(1)
        print(f"  received TaskListMessage: {len(tasks)} thread(s)")
        if len(tasks) == 0:
            print("FAIL: TaskListMessage reported zero threads")
            sys.exit(1)
        for t in tasks:
            print(f"    {t['name']:<20} state={t['state']:<12} prio={t['priority']:<3} "
                  f"stack={t['stackUsedCurrentBytes']}/{t['stackSizeBytes']}B "
                  f"(hwm {t['stackUsedHighWaterBytes']}B) "
                  f"runCount={t['runCount']} cpu={t['cpuPercent']:.1f}%")
            if t["stackUsedCurrentBytes"] > t["stackSizeBytes"] or t["stackUsedHighWaterBytes"] > t["stackSizeBytes"]:
                print(f"FAIL: thread {t['name']!r} reports stack usage exceeding its own stack size "
                      "-- ThreadX stack-analyze data looks corrupted")
                sys.exit(1)
    print("  WebSocket OK (BestBinaryBuffer request/response round trip verified)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", required=True, help="Device hostname or IP, e.g. factory-box-XXXXXX.local")
    parser.add_argument("--port", type=int, default=HTTPS_PORT, help="HTTPS/WSS port (default: 443, same port serves both)")
    parser.add_argument("--ca-cert", required=True,
                         help="Path to the trust anchor (PEM) -- the private CA's certificate (recommended, "
                              "enables real chain+hostname verification), or the device's own leaf "
                              "certificate (e.g. the board archive's factory-box-<id>.pem.crt).")
    args = parser.parse_args()

    ssl_ctx = build_ssl_context(args.ca_cert)
    try:
        test_https(args.host, args.port, ssl_ctx)
        test_websocket(args.host, args.port, ssl_ctx)
    except (OSError, ssl.SSLError) as e:
        print(f"\nFEHLER: {e}")
        sys.exit(1)

    print("\nAll checks passed.")


if __name__ == "__main__":
    main()
