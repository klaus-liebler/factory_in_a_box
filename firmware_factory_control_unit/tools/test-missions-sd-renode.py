#!/usr/bin/env python3
"""RoArm missions on the SD card, end to end in Renode (no hardware): the FCU firmware of
hardware revision 3 runs on rtoos' fcu_rev3-net.resc (STM32H743, TAP adapter "rtoos-tap",
rtoos/emulation/renode/setup-tap.ps1) with a fresh FAT16 card image in SDMMC1; over wss:// it
saves, lists, reads, overwrites and deletes missions (WsProtocol roarm namespace).

  cmake --build --preset Release-Rev3
  python tools/test-missions-sd-renode.py [--elf build/Release-Rev3/firmware_factory_control_unit.elf]

The chip UID in the emulated system memory is set to the board's (generated/device_ids.hh), so the
firmware's identity check passes. Exit code 0 = pass.
"""
import argparse
import base64
import os
import re
import socket
import ssl
import struct
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--elf", type=Path, default=ROOT / "build/Release-Rev3/firmware_factory_control_unit.elf")
ap.add_argument("--renode", default="C:/opt/renode/renode.exe")
args = ap.parse_args()
rtoos = ROOT / "rtoos"
sys.path.insert(0, str(rtoos / "emulation"))
import net_test  # noqa: E402  (DHCP/DNS/SNTP servers on the TAP side, WebSocket frames)

ids = (ROOT / "generated/device_ids.hh").read_text(encoding="utf-8")
uid = [int(re.search(rf"DEVICE_CHIP_UID_W{i} = 0x([0-9A-Fa-f]+)", ids).group(1), 16) for i in range(3)]
elf = args.elf
image = Path(tempfile.mkdtemp()) / "card.img"
SECTORS, SPC, RESERVED, FATS, ROOT_ENTRIES, FAT_SECTORS = 131072, 4, 4, 2, 512, 128
boot = bytearray(512)
boot[0:3] = b"\xEB\x3C\x90"
boot[3:11] = b"MSDOS5.0"
struct.pack_into("<HBHBHHBHHHII", boot, 11, 512, SPC, RESERVED, FATS, ROOT_ENTRIES, 0, 0xF8, FAT_SECTORS, 63, 255, 0,
                 SECTORS)
struct.pack_into("<BBBI11s8s", boot, 36, 0x80, 0, 0x29, 0x12345678, b"FCUTEST    ", b"FAT16   ")
boot[510:512] = b"\x55\xAA"
with open(image, "wb") as f:
    f.truncate(SECTORS * 512)
    f.write(boot)
    for i in range(FATS):
        f.seek((RESERVED + i * FAT_SECTORS) * 512)
        f.write(b"\xF8\xFF\xFF\xFF")

with socket.socket() as s:
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
cmds = "; ".join([
    f"$bin=@{elf.as_posix()}",
    f"include @{(rtoos / 'emulation/renode/fcu_rev3-net.resc').as_posix()}",
    f"sysbus WriteDoubleWord 0x1FF1E800 {uid[0]:#x}",
    f"sysbus WriteDoubleWord 0x1FF1E804 {uid[1]:#x}",
    f"sysbus WriteDoubleWord 0x1FF1E808 {uid[2]:#x}",
    f"sysbus.sdmmc1 InsertCard @{image.as_posix()} false",
    f'emulation CreateServerSocketTerminal {port} "console" false',
    "connector Connect sysbus.usart3 console",
    "start",
])
stop = threading.Event()
net_test.start_servers(stop, [], [])
proc = subprocess.Popen([args.renode, "--disable-gui", "--plain", "--port", "-1", "-e", cmds],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL)
log = []


def pump(sock):
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


def check(name, ok, detail=""):
    global failures
    print(f"  [{'ok' if ok else 'FAIL'}] {name}{(': ' + detail) if detail else ''}")
    failures += 0 if ok else 1


def wait_log(text, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if any(text in line for line in log):
            return True
        time.sleep(0.2)
    return False


try:
    check("SD card mounted", wait_log("missions: SD card mounted", 60))
    check("DHCP lease", wait_log("dhcp: bound", 60))
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    tls = ctx.wrap_socket(socket.create_connection((net_test.DEVICE_IP, 443), timeout=30), server_hostname="fcu.local")
    key = base64.b64encode(os.urandom(16)).decode()
    tls.sendall((f"GET /ws HTTP/1.1\r\nHost: fcu.local\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                 f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\nOrigin: https://fcu.local\r\n\r\n").encode())
    head = b""
    while b"\r\n\r\n" not in head:
        head += tls.recv(1024)
    head, rest = head.split(b"\r\n\r\n", 1)
    check("WebSocket upgrade", b" 101 " in head.split(b"\r\n")[0])
    reader = net_test.WsReader(tls)
    reader.buf = rest
    NS = 2
    req_id = [100]

    def call(type_id, body, answer_type):
        req_id[0] += 1
        net_test.ws_send(tls, struct.pack("<HHH", NS, type_id, req_id[0]) + body)
        tls.settimeout(20)
        end = time.monotonic() + 20
        while time.monotonic() < end:
            opcode, p = reader.frame()
            if opcode == 2 and len(p) >= 6 and struct.unpack_from("<HHH", p) == (NS, answer_type, req_id[0]):
                return p[6:]
        raise TimeoutError(f"no answer {answer_type}")

    def cstr(b, pos):
        end = b.index(0, pos)
        return b[pos:end].decode(), end + 1

    def save(index, name, steps):
        body = struct.pack("<H", index) + name.encode() + b"\0" + struct.pack("<H", len(steps)) + b"".join(steps)
        r = call(12, body, 13)
        return r[0] == 1, struct.unpack_from("<h", r, 1)[0]

    def listing():
        r = call(8, b"", 9)
        count, = struct.unpack_from("<H", r, 0)
        pos, out = 2, []
        for _ in range(count):
            cls, idx = struct.unpack_from("<HH", r, pos)
            name, pos = cstr(r, pos + 4)
            out.append((idx, name))
        return out

    delay = struct.pack("<HH", 4, 500)          # DelayStep 500 ms
    gpio = struct.pack("<HBB", 3, 1, 1)          # GpioStep gpio 1 on
    ok, err = save(7, "erste", [delay, gpio])
    check("save mission 7", ok, f"error {err}")
    ok, err = save(3, "zweite", [delay])
    check("save mission 3", ok, f"error {err}")
    lst = listing()
    check("list: 3 and 7", lst == [(3, "zweite"), (7, "erste")], str(lst))
    r = call(10, struct.pack("<H", 7), 11)
    found = r[0] == 1
    idx, = struct.unpack_from("<H", r, 1)
    name, pos = cstr(r, 3)
    count, = struct.unpack_from("<H", r, pos)
    check("get mission 7", found and idx == 7 and name == "erste" and count == 2, f"{found} {idx} {name} {count}")
    ok, err = save(7, "ueberschrieben", [gpio])
    check("overwrite mission 7", ok, f"error {err}")
    lst = listing()
    check("list after overwrite", lst == [(3, "zweite"), (7, "ueberschrieben")], str(lst))
    r = call(14, struct.pack("<H", 3), 15)
    check("delete mission 3", r[0] == 1)
    lst = listing()
    check("list after delete", lst == [(7, "ueberschrieben")], str(lst))
    r = call(10, struct.pack("<H", 3), 11)
    check("get deleted mission 3: not found", r[0] == 0)
finally:
    stop.set()
    proc.kill()
    proc.wait()
print("--- firmware log (missions):")
for line in log:
    if "missions" in line or "ERR" in line:
        print("  " + line)
print("RESULT:", "PASS" if failures == 0 else f"FAIL ({failures})")
sys.exit(1 if failures else 0)
