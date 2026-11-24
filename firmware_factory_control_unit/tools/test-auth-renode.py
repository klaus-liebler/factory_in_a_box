#!/usr/bin/env python3
"""End-to-end test of the web UI login (src/auth.hpp, docs/security.md) against the FCU firmware in
Renode on the TAP adapter "rtoos-tap" (rtoos/emulation/renode/setup-tap.ps1), like pentest-renode.py:

  static app      GET / needs no login; GET /api/session without a cookie: 401
  one-time pw     the label password (board archive) logs in, but only for the password change:
                  the WebSocket is refused until it is changed; the cookie is HttpOnly, Secure,
                  SameSite=Strict
  login cost      the PBKDF2 check runs in background steps (time of a login over the network)
  roles           admin creates an observer; the observer reads registers, may not write them
                  (0x102), may not manage users (403); the admin over the LAN may not download a
                  PLC program (physical presence: 0x103); /api/recover over the LAN: 403
  tokens          a forged token, a token after logout and a cross-origin upgrade are refused;
                  a login that brings a cookie gets a NEW token and the old one is dead (fixation)
  throttle        wrong passwords: 401, then 429 with Retry-After, without stalling the device
  audit           the console shows the auth: lines

  cmake --build --preset rev3 && python tools/test-auth-renode.py [--revision 1]

--open-ethernet: the firmware was built with WebAuth:Modes:Ethernet = Open (docs/security.md "Login mode
per interface"). Then: /api/session without a cookie is an open programmer session, the WebSocket opens
without login, registers are read and written, a PLC download still needs presence (0x103), user
management still needs a login (401), the label password still logs in, and the console audits it.

Run it unsandboxed (TAP). Exit code 0 = pass.
"""
import argparse
import re
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
import fcu_auth as fa  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--revision", type=int, choices=[1, 3], default=3)
ap.add_argument("--elf", type=Path)
ap.add_argument("--renode", default="C:/opt/renode/renode.exe")
ap.add_argument("--open-ethernet", action="store_true", help="firmware built with WebAuth:Modes:Ethernet = Open")
args = ap.parse_args()
rtoos = ROOT / "rtoos"
sys.path.insert(0, str(rtoos / "emulation"))
import net_test  # noqa: E402

DEV = net_test.DEVICE_IP
elf = args.elf or ROOT / f"build/rev{args.revision}/firmware_factory_control_unit.elf"
resc = "fcu_rev3-net.resc" if args.revision == 3 else "fcu-net.resc"
uid_at = 0x1FF1E800 if args.revision == 3 else 0x08FFF800
ids = (ROOT / "generated/device_ids.hh").read_text(encoding="utf-8")
uid = [int(re.search(rf"DEVICE_CHIP_UID_W{i} = 0x([0-9A-Fa-f]+)", ids).group(1), 16) for i in range(3)]

with socket.socket() as s:
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
cmds = "; ".join([
    f"$bin=@{elf.as_posix()}",
    f"include @{(rtoos / 'emulation/renode' / resc).as_posix()}",
    *[f"sysbus WriteDoubleWord {uid_at + 4 * i:#x} {uid[i]:#x}" for i in range(3)],
    f'emulation CreateServerSocketTerminal {port} "console" false',
    "connector Connect sysbus.usart3 console",
    "start",
])
stop = threading.Event()
net_test.start_servers(stop, [], [])
proc = subprocess.Popen([args.renode, "--disable-gui", "--plain", "--port", "-1", "-e", cmds],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL)
log: list[str] = []


def pump(sock: socket.socket) -> None:
    buf = b""
    while True:
        try:
            d = sock.recv(4096)
        except OSError:
            return
        if not d:
            return
        buf += d
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            log.append(line.decode(errors="replace").rstrip("\r"))


deadline = time.monotonic() + 60
while True:
    try:
        con = socket.create_connection(("127.0.0.1", port), timeout=1)
        break
    except OSError:
        if time.monotonic() > deadline:
            proc.kill()
            sys.exit("no console")
        time.sleep(0.2)
con.settimeout(None)
threading.Thread(target=pump, args=(con,), daemon=True).start()

failures = 0


def check(name: str, ok: bool, detail: str = "") -> bool:
    global failures
    print(f"  [{'ok' if ok else 'FAIL'}] {name}{(': ' + detail) if detail else ''}", flush=True)
    failures += 0 if ok else 1
    return ok


def wait_log(text: str, seconds: float) -> bool:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if any(text in line for line in log):
            return True
        time.sleep(0.2)
    return False


def upgrade_status(cookie: str | None, origin: str | None = "https://fcu.local") -> int:
    try:
        fa.ws_open(DEV, cookie, origin).close()
        return 101
    except OSError as e:
        m = re.search(r"upgrade (\d+)", str(e))
        return int(m.group(1)) if m else 0


MODBUS, PLC = 4, 7
GET_REGISTERS, WRITE_HOLDING, UPLOAD = 1, 3, 5
E_ROLE, E_PRESENCE = 0x102, 0x103


def scenario() -> None:
    label = fa.label_password()
    print("-- static app and anonymous API")
    r = fa.request(DEV, "GET", "/")
    check("GET / without login", r.status == 200, str(r.status))
    check("GET /api/session without cookie: 401", fa.request(DEV, "GET", "/api/session").status == 401)
    check("WebSocket without cookie: 401", upgrade_status(None) == 401)

    print("-- one-time password")
    r = fa.login(DEV, "admin", label)
    check("label password logs in", r.status == 200, f"{r.status} {r.body[:80]!r}")
    check("login takes background steps, not seconds", r.seconds < 5, f"{r.seconds * 1000:.0f} ms")
    print(f"     login round trip: {r.seconds * 1000:.0f} ms (emulator)")
    cookie = r.cookie()
    flags = r.header("Set-Cookie") or ""
    check("cookie HttpOnly; Secure; SameSite=Strict; Path=/",
          all(f in flags for f in ("HttpOnly", "Secure", "SameSite=Strict", "Path=/")), flags)
    check("token: 43 base64url characters", bool(cookie and re.fullmatch(r"[A-Za-z0-9_-]{43}", cookie)), str(cookie))
    check("must change the password", r.json().get("mustChangePassword") is True, r.body.decode(errors="replace"))
    check("WebSocket refused until the password is changed", upgrade_status(cookie) == 401)
    c = fa.request(DEV, "POST", "/api/password", {"current": label, "new": "short"}, cookie=cookie)
    check("too short a new password refused", c.status == 400, f"{c.status} {c.body!r}")
    c = fa.request(DEV, "POST", "/api/password", {"current": label, "new": fa.TEST_PASSWORD}, cookie=cookie)
    check("password changed", c.status == 200, f"{c.status} {c.body!r}")
    check("the label password no longer works", fa.login(DEV, "admin", label).status == 401)
    s = fa.request(DEV, "GET", "/api/session", cookie=cookie)
    check("the session that changed it goes on, no longer must change", s.status == 200 and s.json().get("mustChangePassword") is False,
          s.body.decode(errors="replace"))

    print("-- roles and presence")
    ws = fa.ws_open(DEV, cookie)
    a = fa.ws_request(ws, MODBUS, GET_REGISTERS, 1)
    check("admin reads registers", fa.ws_error_code(a) is None and a[1] == 2)
    a = fa.ws_request(ws, MODBUS, WRITE_HOLDING, 2, struct.pack("<HH", 0, 0))
    check("admin may write registers (empty write: success=false)", fa.ws_error_code(a) is None, a[4].hex())
    a = fa.ws_request(ws, PLC, UPLOAD, 3, struct.pack("<IIH", 0, 0, 0))
    check("PLC download over the LAN needs presence (0x103)", fa.ws_error_code(a) == E_PRESENCE, str(fa.ws_error_code(a)))
    ws.close()
    u = fa.request(DEV, "POST", "/api/users", {"action": "add", "user": "watcher", "role": "observer",
                                               "password": "Watcher-Passwort-9"}, cookie=cookie)
    check("admin adds an observer", u.status == 200, f"{u.status} {u.body!r}")
    lst = fa.request(DEV, "GET", "/api/users", cookie=cookie)
    check("user list shows it with must-change", b'"user":"watcher","role":"observer","mustChangePassword":true' in lst.body,
          lst.body.decode(errors="replace"))
    o = fa.login(DEV, "watcher", "Watcher-Passwort-9")
    ocookie = o.cookie()
    c = fa.request(DEV, "POST", "/api/password", {"current": "Watcher-Passwort-9", "new": "Watcher-Eigenes-10"}, cookie=ocookie)
    check("observer sets an own password", o.status == 200 and c.status == 200, f"{o.status}/{c.status}")
    ows = fa.ws_open(DEV, ocookie)
    a = fa.ws_request(ows, MODBUS, GET_REGISTERS, 4)
    check("observer reads registers", fa.ws_error_code(a) is None)
    a = fa.ws_request(ows, MODBUS, WRITE_HOLDING, 5, struct.pack("<HH", 0, 0))
    check("observer may not write registers (0x102)", fa.ws_error_code(a) == E_ROLE, str(fa.ws_error_code(a)))
    a = fa.ws_request(ows, PLC, UPLOAD, 6, struct.pack("<IIH", 0, 0, 0))
    check("observer may not download (0x102)", fa.ws_error_code(a) == E_ROLE, str(fa.ws_error_code(a)))
    ows.close()
    esc = fa.request(DEV, "POST", "/api/users", {"action": "role", "user": "watcher", "role": "admin"}, cookie=ocookie)
    check("observer cannot make itself admin (403)", esc.status == 403, f"{esc.status} {esc.body!r}")
    check("observer cannot list users (403)", fa.request(DEV, "GET", "/api/users", cookie=ocookie).status == 403)
    rc = fa.request(DEV, "POST", "/api/recover", {})
    check("password recovery over the LAN refused (403)", rc.status == 403, f"{rc.status} {rc.body!r}")
    rp = fa.request(DEV, "POST", "/api/users", {"action": "password", "user": "watcher", "password": "Neues-Passwort-11"}, cookie=cookie)
    check("password reset of another user over the LAN needs presence (403)", rp.status == 403, f"{rp.status} {rp.body!r}")

    print("-- tokens")
    forged = cookie[:-1] + ("A" if cookie[-1] != "A" else "B")
    check("forged token: WebSocket 401", upgrade_status(forged) == 401)
    check("forged token: /api/session 401", fa.request(DEV, "GET", "/api/session", cookie=forged).status == 401)
    check("cross-origin upgrade with a valid cookie: 403", upgrade_status(cookie, "https://evil.example") == 403)
    x = fa.request(DEV, "POST", "/api/login", {"user": "admin", "password": fa.TEST_PASSWORD}, origin="https://evil.example")
    check("cross-origin login POST: 403", x.status == 403, str(x.status))
    f = fa.login(DEV, "admin", fa.TEST_PASSWORD, cookie=cookie)
    check("login with a cookie gets a new token (fixation)", f.status == 200 and f.cookie() not in (None, cookie))
    check("the token brought along is dead", fa.request(DEV, "GET", "/api/session", cookie=cookie).status == 401)
    cookie = f.cookie()
    ws = fa.ws_open(DEV, cookie)
    out = fa.request(DEV, "POST", "/api/logout", {}, cookie=cookie)
    check("logout clears the cookie", out.status == 200 and "Max-Age=0" in (out.header("Set-Cookie") or ""))
    check("token after logout: 401", fa.request(DEV, "GET", "/api/session", cookie=cookie).status == 401)
    try:
        closed = False
        end = time.monotonic() + 5
        while time.monotonic() < end and not closed:
            op, p = fa.ws_recv(ws, 3)
            closed = op == 8
        check("the open WebSocket of that login is closed (1008)", closed and p[:2] == b"\x03\xf0", p[:2].hex() if closed else "open")
    except OSError as e:
        check("the open WebSocket of that login is closed (1008)", "closed" in str(e) or "timed out" not in str(e), str(e))
    ws.close()

    print("-- throttle")
    codes = []
    for _ in range(6):
        codes.append(fa.login(DEV, "admin", "wrong password").status)
    check("wrong passwords: 401 first, then 429", codes[:3] == [401, 401, 401] and 429 in codes, str(codes))
    t = fa.login(DEV, "admin", fa.TEST_PASSWORD)
    check("while throttled even the right password waits (429 + Retry-After)", t.status == 429 and t.header("Retry-After") is not None,
          f"{t.status} Retry-After {t.header('Retry-After')}")
    r = fa.request(DEV, "GET", "/")
    check("the device keeps serving while throttling", r.status == 200)

    print("-- audit")
    time.sleep(1)
    for text in ("auth: admin logged in", "auth: login of admin from", "failed", "auth: password of admin changed",
                 "auth: admin added user watcher", "refused: physical presence required", "refused: role too low",
                 "throttled", "logged out"):
        check(f"console: {text!r}", any(text in line for line in log))


def open_scenario() -> None:
    """WebAuth:Modes:Ethernet = Open: programmer without login, presence and accounts unchanged."""
    print("-- open Ethernet")
    s = fa.request(DEV, "GET", "/api/session")
    j = s.json() if s.status == 200 else {}
    check("GET /api/session without cookie: open programmer session",
          s.status == 200 and j.get("open") is True and j.get("role") == "programmer" and j.get("mustChangePassword") is False,
          f"{s.status} {s.body[:120]!r}")
    check("WebSocket without cookie: 101", upgrade_status(None) == 101)
    check("cross-origin upgrade still refused: 403", upgrade_status(None, "https://evil.example") == 403)
    ws = fa.ws_open(DEV, None)
    a = fa.ws_request(ws, MODBUS, GET_REGISTERS, 1)
    check("reads registers without login", fa.ws_error_code(a) is None)
    a = fa.ws_request(ws, MODBUS, WRITE_HOLDING, 2, struct.pack("<HH", 0, 0))
    check("writes registers without login", fa.ws_error_code(a) is None)
    a = fa.ws_request(ws, PLC, UPLOAD, 3, struct.pack("<IIH", 0, 0, 0))
    check("PLC download over the open LAN still needs presence (0x103)", fa.ws_error_code(a) == E_PRESENCE, str(fa.ws_error_code(a)))
    ws.close()
    check("user management without login: 401", fa.request(DEV, "GET", "/api/users").status == 401)
    check("password recovery over the LAN still refused (403)", fa.request(DEV, "POST", "/api/recover", {}).status == 403)
    r = fa.login(DEV, "admin", fa.label_password())
    check("the label password still logs in (admin)", r.status == 200 and r.json().get("mustChangePassword") is True,
          f"{r.status} {r.body[:80]!r}")
    print("-- audit")
    time.sleep(1)
    for text in ("auth: Ethernet is OPEN", "without login: open interface", "refused: physical presence required"):
        check(f"console: {text!r}", any(text in line for line in log))


try:
    check("boot: DHCP lease", wait_log("dhcp: bound", 120))
    check("boot: accounts", wait_log("auth: ", 10), next((l for l in log if "auth: " in l), ""))
    try:
        open_scenario() if args.open_ethernet else scenario()
    except Exception as e:  # noqa: BLE001 - a scenario that could not run is a failure, report it
        check("scenario ran", False, f"{type(e).__name__}: {e}")
    bad = [line for line in log if re.search(r"fault|contract|assert|panic|stack overflow", line, re.I)]
    check("no fault, contract violation or assertion in the console", not bad, "; ".join(bad[:3]))
finally:
    stop.set()
    proc.kill()
    proc.wait()
print("--- firmware log (auth):")
for line in log:
    if "auth:" in line:
        print("  " + line)
print("RESULT:", "PASS" if failures == 0 else f"FAIL ({failures})")
sys.exit(1 if failures else 0)
