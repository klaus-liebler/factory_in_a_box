"""Login helpers for the FCU test tools (docs/security.md): the board's one-time web password from
the board archive, HTTPS API calls, the session cookie and a logged-in WebSocket.

The password is read where the builder keeps it: <BoardStorage:BoardsDir>/<board id>/board_info.json
("webAdminPassword"), the board id is the hostname suffix and chip UID of generated/device-ids.json.
"""
from __future__ import annotations

import base64
import json
import os
import re
import socket
import ssl
import struct
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
COOKIE = "fcu_session"
TEST_PASSWORD = "Renode-Test-Passwort-1"  # what the tests change the one-time password to


def board_archive_record() -> dict:
    ids = json.loads((ROOT / "generated/device-ids.json").read_text(encoding="utf-8"))
    settings = ROOT / "builder/appsettings.json"
    if not settings.exists():
        settings = ROOT / "builder/appsettings.json.template"
    boards = Path(json.loads(settings.read_text(encoding="utf-8"))["BoardStorage"]["BoardsDir"])
    board_id = f"{ids['hostname'].rsplit('-', 1)[1]}_{ids['chipUid'].lower()}"
    return json.loads((boards / board_id / "board_info.json").read_text(encoding="utf-8"))


def label_password() -> str:
    pw = board_archive_record().get("webAdminPassword")
    if not pw:
        raise SystemExit("no webAdminPassword in the board archive: run the builder (GenerateWebCredentials)")
    return pw


def tls_ctx() -> ssl.SSLContext:
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx


class Response:
    def __init__(self, status: int, head: bytes, body: bytes, seconds: float):
        self.status, self.head, self.body, self.seconds = status, head, body, seconds

    def header(self, name: str) -> str | None:
        for line in self.head.split(b"\r\n")[1:]:
            k, _, v = line.partition(b":")
            if k.strip().lower() == name.lower().encode():
                return v.strip().decode()
        return None

    def cookie(self) -> str | None:
        c = self.header("Set-Cookie")
        m = re.match(rf"{COOKIE}=([^;]*)", c or "")
        return m.group(1) if m else None

    def json(self) -> dict:
        try:
            return json.loads(self.body or b"{}")
        except ValueError:
            return {}


def request(dev: str, method: str, path: str, body: dict | bytes | None = None, cookie: str | None = None,
            origin: str | None = "https://fcu.local", timeout: float = 30) -> Response:
    data = body if isinstance(body, bytes) else (json.dumps(body).encode() if body is not None else b"")
    head = f"{method} {path} HTTP/1.1\r\nHost: fcu.local\r\nAccept-Encoding: br\r\nConnection: close\r\n"
    if origin:
        head += f"Origin: {origin}\r\n"
    if cookie:
        head += f"Cookie: {COOKIE}={cookie}\r\n"
    if method == "POST":
        head += f"Content-Type: application/json\r\nContent-Length: {len(data)}\r\n"
    raw = head.encode() + b"\r\n" + data
    with tls_ctx().wrap_socket(socket.create_connection((dev, 443), timeout=timeout), server_hostname="fcu.local") as s:
        s.settimeout(timeout)
        t0 = time.monotonic()
        s.sendall(raw)
        buf = b""
        while True:
            try:
                d = s.recv(4096)
            except (OSError, ssl.SSLError):
                break
            if not d:
                break
            buf += d
            if b"\r\n\r\n" in buf:
                h, _, rest = buf.partition(b"\r\n\r\n")
                m = re.search(rb"Content-Length: (\d+)", h, re.I)
                if m and len(rest) >= int(m.group(1)):
                    break
        seconds = time.monotonic() - t0
    h, _, b = buf.partition(b"\r\n\r\n")
    m = re.match(rb"HTTP/1\.1 (\d{3})", h)
    return Response(int(m.group(1)) if m else 0, h, b, seconds)


def login(dev: str, user: str, password: str, cookie: str | None = None) -> Response:
    return request(dev, "POST", "/api/login", {"user": user, "password": password}, cookie=cookie)


def admin_session(dev: str) -> str:
    """A logged-in admin session: with the label password (and the forced change to TEST_PASSWORD), or
    with TEST_PASSWORD if that change happened before in this run."""
    r = login(dev, "admin", TEST_PASSWORD)
    if r.status == 200:
        return r.cookie() or ""
    r = login(dev, "admin", label_password())
    if r.status != 200:
        raise OSError(f"login with the label password: {r.status} {r.body!r}")
    cookie = r.cookie()
    c = request(dev, "POST", "/api/password", {"current": label_password(), "new": TEST_PASSWORD}, cookie=cookie)
    if c.status != 200:
        raise OSError(f"password change: {c.status} {c.body!r}")
    return cookie or ""


def ws_open(dev: str, cookie: str | None, origin: str | None = "https://fcu.local", timeout: float = 20) -> ssl.SSLSocket:
    s = tls_ctx().wrap_socket(socket.create_connection((dev, 443), timeout=timeout), server_hostname="fcu.local")
    key = base64.b64encode(os.urandom(16)).decode()
    o = f"Origin: {origin}\r\n" if origin else ""
    c = f"Cookie: {COOKIE}={cookie}\r\n" if cookie else ""
    s.settimeout(timeout)
    s.sendall((f"GET /ws HTTP/1.1\r\nHost: fcu.local\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
               f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n{o}{c}\r\n").encode())
    data = b""
    while b"\r\n\r\n" not in data:
        d = s.recv(4096)
        if not d:
            break
        data += d
    m = re.match(rb"HTTP/1\.1 (\d{3})", data)
    status = int(m.group(1)) if m else 0
    if status != 101:
        s.close()
        raise OSError(f"upgrade {status}")
    return s


def ws_send(s: ssl.SSLSocket, payload: bytes) -> None:
    mask = os.urandom(4)
    n = len(payload)
    head = bytes([0x82]) + (bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + struct.pack(">H", n))
    s.sendall(head + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))


def ws_recv(s: ssl.SSLSocket, timeout: float = 10) -> tuple[int, bytes]:
    """The next data frame (opcode, payload); (8, payload) for a close."""
    s.settimeout(timeout)

    def exact(n: int) -> bytes:
        b = b""
        while len(b) < n:
            d = s.recv(n - len(b))
            if not d:
                raise OSError("closed")
            b += d
        return b

    while True:
        h = exact(2)
        n = h[1] & 0x7F
        if n == 126:
            n = struct.unpack(">H", exact(2))[0]
        elif n == 127:
            n = struct.unpack(">Q", exact(8))[0]
        payload = exact(n)
        if h[0] & 0x0F in (1, 2, 8):
            return h[0] & 0x0F, payload


def ws_request(s: ssl.SSLSocket, ns: int, type_id: int, corr: int, body: bytes = b"", timeout: float = 10) -> tuple[int, int, int, int, bytes]:
    """Sends a WsProtocol request, returns the answer with this correlation id: (ns, type, corr, flags, payload)."""
    ws_send(s, struct.pack("<HHHH", ns, type_id, corr, 0) + body)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        op, p = ws_recv(s, timeout)
        if op == 8:
            raise OSError(f"closed {p[:2].hex()}")
        if len(p) >= 8:
            a, b, c, f = struct.unpack_from("<HHHH", p)
            if c == corr:
                return a, b, c, f, p[8:]
    raise OSError("no answer")


def ws_error_code(answer: tuple[int, int, int, int, bytes]) -> int | None:
    """The ErrorCode of a FLAG_ERROR answer, None for a normal answer."""
    _, _, _, flags, payload = answer
    if flags & 1 and len(payload) >= 2:
        return struct.unpack_from("<H", payload)[0]
    return None
