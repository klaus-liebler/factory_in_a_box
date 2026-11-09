#!/usr/bin/env python3
"""The IEC 61131-3 PLC of the FCU firmware, end to end in Renode (no hardware): the firmware of
hardware revision 3 (STM32H743, rtoos' fcu_rev3-net.resc) or 2 (STM32H563, fcu-net.resc) runs on the
TAP adapter "rtoos-tap" (rtoos/emulation/renode/setup-tap.ps1); over wss:// (WsProtocol namespace
"plc") and Modbus TCP:

  - fieldbus mode: Modbus writes the outputs as before
  - an image built for another register map (I/O map hash) is refused
  - download of tools/plc-renode/test.img in chunks, commit into the flash slot
  - control source IEC: the outputs go to the safe state, Modbus writes to them and to the command
    registers are refused with exception 0x02
  - a command register (Stepper1StartHoming) through the FCU library block FCU_StepperHome: the
    start reaches the firmware's homing, its answer the block (the emulator has no TMC2209: 3)
  - RUN: the program writes its outputs into the holding registers (32-bit value high word first),
    reads inputs, its variables are readable online
  - autostart: after a software reset the program runs again by itself - no browser, no download
  - STOP: safe state; control source fieldbus: Modbus writes the outputs again
  - the terminal command `plc`: the state; `plc source fieldbus`, `plc autostart off` through the PLC thread
  - the project source on the SD card (a FAT16 image): none at first, stored in chunks, read back the
    same, a chunk out of order refused, still there after the reset

  cmake --build --preset Release-Rev3 && python tools/test-plc-renode.py
  cmake --build --preset Release      && python tools/test-plc-renode.py --revision 2

The image comes from `npm run plc:corpus` (web/src/plc/renode-fixture.test.ts). Exit code 0 = pass.
"""
import argparse
import base64
import json
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
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--revision", type=int, choices=[2, 3], default=3)
ap.add_argument("--elf", type=Path, help="default: build/Release-Rev3 (revision 3) or build/Release (2)")
ap.add_argument("--old-mission-store", action="store_true",
                help="revision 2: the flash area still holds the littlefs mission store of the old firmware "
                     "(no SD card): the PLC must refuse to write it")
ap.add_argument("--renode", default="C:/opt/renode/renode.exe")
args = ap.parse_args()
elf = args.elf or ROOT / f"build/{'Release-Rev3' if args.revision == 3 else 'Release'}/firmware_factory_control_unit.elf"
resc = "fcu_rev3-net.resc" if args.revision == 3 else "fcu-net.resc"
uid_at = 0x1FF1E800 if args.revision == 3 else 0x08FFF800  # STM32H743 / STM32H563 unique ID
rtoos = ROOT / "rtoos"
sys.path.insert(0, str(rtoos / "emulation"))
import net_test  # noqa: E402  (DHCP/DNS/SNTP servers on the TAP side, WebSocket frames, Modbus TCP)

image = (ROOT / "tools/plc-renode/test.img").read_bytes()
meta = json.loads((ROOT / "tools/plc-renode/test.json").read_text(encoding="utf-8"))
assert zlib.crc32(image) == meta["crc"], "test.img and test.json do not match: npm run plc:corpus"

ids = (ROOT / "generated/device_ids.hh").read_text(encoding="utf-8")
uid = [int(re.search(rf"DEVICE_CHIP_UID_W{i} = 0x([0-9A-Fa-f]+)", ids).group(1), 16) for i in range(3)]



def fat16_image(path, total_sectors=65536):
    """An empty FAT16 file system without a partition table (FileX mounts sector 0 as the boot
    sector): 512-byte sectors, 4 per cluster, 2 FATs, 512 root entries - 32 MB."""
    spc, reserved, fats, root_entries = 4, 1, 2, 512
    root_sectors = root_entries * 32 // 512
    fat_sectors = 64  # (65536 - 1 - 32) / 4 clusters, 2 bytes each: 63.96 sectors
    boot = bytearray(512)
    boot[0:3] = b"\xEB\x3C\x90"
    boot[3:11] = b"RTOOSFAT"
    struct.pack_into("<HBHBHHBHHHII", boot, 11, 512, spc, reserved, fats, root_entries,
                     0 if total_sectors > 0xFFFF else total_sectors, 0xF8, fat_sectors, 63, 255, 0,
                     total_sectors if total_sectors > 0xFFFF else 0)
    struct.pack_into("<BBBI11s8s", boot, 36, 0x80, 0, 0x29, 0x1234ABCD, b"NO NAME    ", b"FAT16   ")
    boot[510:512] = b"\x55\xAA"
    fat = bytearray(fat_sectors * 512)
    struct.pack_into("<HH", fat, 0, 0xFFF8, 0xFFFF)
    with open(path, "wb") as f:
        f.truncate(total_sectors * 512)
        f.write(boot)
        f.seek(reserved * 512)
        f.write(fat * fats)
    assert reserved + fats * fat_sectors + root_sectors < total_sectors


card_dir = tempfile.TemporaryDirectory()
card = Path(card_dir.name) / "card.img"
fat16_image(card)

with socket.socket() as s:
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
cmds = "; ".join([
    f"$bin=@{elf.as_posix()}",
    f"include @{(rtoos / 'emulation/renode' / resc).as_posix()}",
    f"sysbus WriteDoubleWord {uid_at:#x} {uid[0]:#x}",
    f"sysbus WriteDoubleWord {uid_at + 4:#x} {uid[1]:#x}",
    f"sysbus WriteDoubleWord {uid_at + 8:#x} {uid[2]:#x}",
    # the old mission store: a littlefs superblock ("littlefs" at offset 8 of block 0)
    *([f"sysbus WriteDoubleWord {0x081C0008 + 4 * i:#x} {struct.unpack('<I', b'littlefs'[4 * i:4 * i + 4])[0]:#x}"
       for i in range(2)] if args.old_mission_store else []),
    f"sysbus.sdmmc1 InsertCard @{card.as_posix()} true",
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


def wait_log(text, seconds, after=0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if any(text in line for line in log[after:]):
            return True
        time.sleep(0.2)
    return False


class Ws:
    """wss://device/ws with request/response by namespace and type id."""

    def __init__(self):
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        self.tls = ctx.wrap_socket(socket.create_connection((net_test.DEVICE_IP, 443), timeout=30), server_hostname="fcu.local")
        key = base64.b64encode(os.urandom(16)).decode()
        self.tls.sendall((f"GET /ws HTTP/1.1\r\nHost: fcu.local\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                          f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\nOrigin: https://fcu.local\r\n\r\n").encode())
        head = b""
        while b"\r\n\r\n" not in head:
            head += self.tls.recv(1024)
        head, rest = head.split(b"\r\n\r\n", 1)
        if b" 101 " not in head.split(b"\r\n")[0]:
            raise ConnectionError("no WebSocket upgrade")
        self.reader = net_test.WsReader(self.tls)
        self.reader.buf = rest
        self.req = 100

    def call(self, ns, type_id, body, answer_type, timeout=30):
        self.req += 1
        net_test.ws_send(self.tls, struct.pack("<HHH", ns, type_id, self.req) + body)
        self.tls.settimeout(timeout)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            opcode, p = self.reader.frame()
            if opcode == 2 and len(p) >= 6 and struct.unpack_from("<HHH", p) == (ns, answer_type, self.req):
                return p[6:]
        raise TimeoutError(f"no answer {ns}/{answer_type}")

    def close(self):
        try:
            self.tls.close()
        except OSError:
            pass


PLC = 7
STATES = {0: "NO_PROGRAM", 1: "STOP", 2: "RUN", 3: "ERROR"}
CMD = {"run": 1, "stop": 2, "reset_warm": 3, "fieldbus": 5, "iec": 6, "autostart_on": 7, "autostart_off": 8}


def cstr(b, pos):
    end = b.index(0, pos)
    return b[pos:end].decode(), end + 1


def status(ws):
    r = ws.call(PLC, 1, b"", 2)
    state, source, autostart, cycle_ms, cycles, last, mx, img, proj, iomap, trap, trap_code, trap_pc, max_img, data = \
        struct.unpack_from("<BBBHIIIIIIBHIII", r, 0)
    problem, _ = cstr(r, struct.calcsize("<BBBHIIIIIIBHIII"))
    return {"state": STATES[state], "iec": source == 1, "autostart": bool(autostart), "cycles": cycles, "image": img,
            "trap": trap, "problem": problem, "instructions": last}


def ok_message(r):
    msg, _ = cstr(r, 1)
    return r[0] == 1, msg


def control(ws, name):
    return ok_message(ws.call(PLC, 3, struct.pack("<B", CMD[name]), 4))


def term(ws, line):
    """A terminal command line (namespace terminal: CommandRequest 3 -> CommandResponse 4)."""
    r = ws.call(6, 3, line.encode() + b"\0", 4)
    out, _ = cstr(r, 1)
    return r[0] == 1, out


def put_project(ws, data, chunks=None):
    """PutProjectRequest 13 -> 14 in 1 KB chunks (or the given (offset, length) chunks); the last answer."""
    for offset, length in chunks or [(o, min(1024, len(data) - o)) for o in range(0, len(data), 1024)]:
        chunk = data[offset:offset + length]
        ok, msg = ok_message(ws.call(PLC, 13, struct.pack("<IIH", len(data), offset, len(chunk)) + chunk, 14))
        if not ok:
            return False, f"offset {offset}: {msg}"
    return True, ""


def get_project(ws):
    """GetProjectRequest 15 -> 16 until complete: (bytes or None, message)."""
    out = b""
    total = 1
    while len(out) < total:
        r = ws.call(PLC, 15, struct.pack("<I", len(out)), 16)
        ok, total, offset, count = struct.unpack_from("<BIIH", r, 0)
        data = r[11:11 + count]
        msg, _ = cstr(r, 11 + count)
        if not ok:
            return None, msg
        if offset != len(out) or (count == 0 and len(out) < total):
            return None, f"chunk at {offset}, {count} bytes, {len(out)} of {total}"
        out += data
    return out, ""


PROJECT = json.dumps({"format": 1, "name": "Renode",
                      "objects": [{"id": "plc_prg", "name": "PLC_PRG", "kind": "PRG", "lang": "ST",
                                   "decl": "PROGRAM PLC_PRG\nVAR\n" + "".join(f"    v{i} : INT; (* \u00e4 *)\n" for i in range(80)) + "END_VAR",
                                   "body": "v0 := v0 + 1;\n"}],
                      "tasks": [{"name": "MainTask", "intervalCycles": 1, "programs": ["PLC_PRG"]}]},
                     ensure_ascii=False).encode()


def read_var(ws, name, fmt):
    addr = meta["symbols"][name]
    r = ws.call(PLC, 9, struct.pack("<IH", addr, struct.calcsize(fmt)), 10)
    ok, address, count = struct.unpack_from("<BIH", r, 0)
    if not ok:
        return None
    return struct.unpack_from(fmt, r, 7)[0]


def download(ws, image=image):
    total = len(image)
    for offset in range(0, total, 1024):
        chunk = image[offset:offset + 1024]
        ok, msg = ok_message(ws.call(PLC, 5, struct.pack("<IIH", total, offset, len(chunk)) + chunk, 6))
        if not ok:
            return False, f"chunk {offset}: {msg}"
    return ok_message(ws.call(PLC, 7, struct.pack("<II", total, zlib.crc32(image)), 8, timeout=60))


def until(probe, seconds=20, step=0.2):
    """Polls probe() until it returns something truthy (the emulators run slower than real time:
    no fixed waits for emulated time); returns the last value."""
    end = time.monotonic() + seconds
    while True:
        v = probe()
        if v or time.monotonic() > end:
            return v
        time.sleep(step)


def modbus_write(mb, reg, value):
    """None: written, else the exception code."""
    r = mb.write(reg, value)
    return None if r[0] == 0x06 else r[1]


def holding(mb, reg, count=1):
    return mb.read(0x03, reg, count)


CONVEYOR, VALVE2, VALVE3, STEPPER1_TARGET, STEPPER2_TARGET, HOMING1, MISSION = 40, 21, 22, 61, 65, 69, 110

ws = mb = None
if args.old_mission_store:
    if args.revision != 2:
        sys.exit("--old-mission-store: revision 2 only (STM32H563)")
    try:
        check("DHCP lease", wait_log("dhcp: bound", 90))
        check("log: the area still holds the missions", wait_log("plc: the flash still holds RoArm missions", 10))
        ws = Ws()
        st = status(ws)
        check("status names the problem", "RoArm missions" in st["problem"], st["problem"])
        ok, msg = download(ws)
        check("download refused", not ok and "RoArm missions" in msg, msg)
        check("no program loaded", status(ws)["image"] == 0)
        ok, msg = control(ws, "iec")
        check("switching the source is refused too (it would write the area)", not ok, msg)
        check("... and nothing switched: still fieldbus", not status(ws)["iec"])
    finally:
        if ws is not None:
            ws.close()
        stop.set()
        proc.kill()
        proc.wait()
    print("RESULT:", "PASS" if failures == 0 else f"FAIL ({failures})")
    sys.exit(1 if failures else 0)
try:
    check("DHCP lease", wait_log("dhcp: bound", 90))
    check("PLC thread up, no program", wait_log("plc: no program", 30))
    ws = Ws()
    mb = net_test.ModbusClient()

    st = status(ws)
    check("status: NO_PROGRAM, fieldbus, no store problem", st["state"] == "NO_PROGRAM" and not st["iec"] and st["problem"] == "", str(st))
    check("fieldbus mode: Modbus writes the conveyor", modbus_write(mb, CONVEYOR, 500) is None and holding(mb, CONVEYOR) == [500])

    # the same image, built for another register map (I/O map hash changed, CRC fixed up)
    foreign = bytearray(image)
    struct.pack_into("<I", foreign, 24, struct.unpack_from("<I", image, 24)[0] ^ 1)
    struct.pack_into("<I", foreign, 16, zlib.crc32(foreign[20:]))
    ok, msg = download(ws, bytes(foreign))
    check("an image built for another register map is refused", not ok and "register map" in msg, msg)
    check("... and nothing is loaded", status(ws)["image"] == 0)

    # the project source on the SD card (docs/plc-iec61131.md 4.5)
    data, msg = get_project(ws)
    check("project: none on a new card", data is None and "no project stored" in msg, msg)
    ok, msg = put_project(ws, PROJECT)
    check(f"project: stored on the SD card ({len(PROJECT)} bytes, {(len(PROJECT) + 1023) // 1024} chunks)", ok, msg)
    data, msg = get_project(ws)
    check("project: read back the same", data == PROJECT, msg or f"{len(data or b'')} bytes")
    ok, msg = put_project(ws, PROJECT, [(0, 1024), (2048, 100)])
    check("project: a chunk out of order is refused", not ok and "out of order" in msg, msg)
    data, msg = get_project(ws)
    check("project: the refused upload left the stored one", data == PROJECT, msg)

    ok, msg = download(ws)
    check("download and commit into the flash slot", ok, msg)
    check("log: program stored", wait_log("plc: program stored", 10))
    st = status(ws)
    check("status: STOP with the image", st["state"] == "STOP" and st["image"] == len(image), str(st))

    ok, msg = control(ws, "iec")
    check("control source IEC", ok, msg)
    check("IEC: outputs in the safe state", holding(mb, CONVEYOR) == [0])
    check("IEC: Modbus write to an output -> exception 02", modbus_write(mb, CONVEYOR, 7) == 0x02)
    check("IEC: the refused write changed nothing", holding(mb, CONVEYOR) == [0])
    check("IEC: Modbus write to a command register -> exception 02 (the program's)", modbus_write(mb, MISSION, 1) == 0x02
          and modbus_write(mb, HOMING1, 1) == 0x02)
    run_mark = len(log)

    ok, msg = control(ws, "run")
    check("RUN", ok, msg)
    until(lambda: status(ws)["cycles"] >= 10)
    st = status(ws)
    check("status: RUN, cycles counting", st["state"] == "RUN" and st["cycles"] >= 10, str(st))
    check("output ConveyorPwm = 123 in the holding register", holding(mb, CONVEYOR) == [123], str(holding(mb, CONVEYOR)))
    check("output Valve2 = TRUE", holding(mb, VALVE2) == [1])
    check("32-bit output Stepper1Target = 70000: high word first", holding(mb, STEPPER1_TARGET, 2) == [1, 4464],
          str(holding(mb, STEPPER1_TARGET, 2)))
    n1 = read_var(ws, "PLC_PRG.n", "<i")
    n2 = until(lambda: (lambda v: v if v is not None and n1 is not None and v > n1 else None)(read_var(ws, "PLC_PRG.n", "<i")))
    check("online value: the cycle counter n grows", n1 is not None and n2 is not None and n2 > n1, f"{n1} -> {n2}")
    t1 = read_var(ws, "PLC_PRG.tick", "<H")
    timer_tick = mb.read(0x04, 10, 1)[0]
    check("input TimerTick reaches the program", t1 is not None and t1 != 0, f"tick {t1}, register {timer_tick}")
    lamps = set()
    until(lambda: lamps.add(holding(mb, VALVE3)[0]) or lamps == {0, 1}, step=0.1)
    check("the TON blinks Valve3", lamps == {0, 1}, str(lamps))
    # the program's homing start went to the register once (no stale answer: the firmware logs this start)
    check("command: the homing started by FCU_StepperHome reaches the firmware",
          wait_log("stepper 1: homing", 20, run_mark))
    until(lambda: read_var(ws, "PLC_PRG.home.Busy", "<B") == 0)
    done, error, error_id = (read_var(ws, f"PLC_PRG.home.{v}", f) for v, f in (("Done", "<B"), ("Error", "<B"), ("ErrorID", "<H")))
    answer = holding(mb, HOMING1)
    check("command: the block has the firmware's answer (no TMC2209 in the emulator: Error 3)",
          done == 0 and error == 1 and error_id == 3 and answer == [3], f"Done {done} Error {error} ErrorID {error_id}, register {answer}")
    time.sleep(1)
    check("command: written only on the edge - the answer stays (no second start)", holding(mb, HOMING1) == [3]
          and sum("stepper 1: homing" in line for line in log[run_mark:]) == 1)
    # FCU_StepperMoveAbsolute on axis 2: the firmware's status says "not referenced" (no TMC2209, no homing)
    until(lambda: read_var(ws, "PLC_PRG.move.Error", "<B") == 1)
    move = [read_var(ws, f"PLC_PRG.move.{v}", f) for v, f in (("Busy", "<B"), ("Error", "<B"), ("ErrorID", "<H"))]
    check("FCU_StepperMoveAbsolute: axis not referenced -> Error, ErrorID 1, the target untouched",
          move == [0, 1, 1] and holding(mb, STEPPER2_TARGET, 2) == [0, 0], f"Busy/Error/ErrorID {move}, target {holding(mb, STEPPER2_TARGET, 2)}")

    ok, msg = control(ws, "autostart_on")
    check("autostart on (journal in the flash slot)", ok, msg)
    # a software reset: the program must come back by itself - no browser, no download
    mark = len(log)
    try:
        ws.call(6, 3, b"reset now\0", 4, timeout=5)
    except (TimeoutError, OSError, ConnectionError):
        pass
    ws.close()
    mb.close()
    ws = mb = None
    check("reset", wait_log("plc: program loaded, control source IEC 61131, autostart on", 60, mark))
    check("after the reset: RUN by itself", wait_log("plc: RUN", 10, mark))
    check("DHCP lease again", wait_log("dhcp: bound", 90, mark))
    time.sleep(1)
    ws = Ws()
    mb = net_test.ModbusClient()
    until(lambda: holding(mb, CONVEYOR) == [123])
    st = status(ws)
    check("status after the reset: RUN, IEC, autostart", st["state"] == "RUN" and st["iec"] and st["autostart"], str(st))
    check("outputs again from the program", holding(mb, CONVEYOR) == [123])
    data, msg = get_project(ws)
    check("project: still on the SD card after the reset", data == PROJECT, msg)

    ok, msg = control(ws, "stop")
    check("STOP", ok, msg)
    check("STOP: outputs in the safe state", holding(mb, CONVEYOR) == [0] and holding(mb, VALVE2) == [0])
    check("STOP: the 32-bit stepper target keeps its value (no new target)", holding(mb, STEPPER1_TARGET, 2) == [1, 4464])
    # the terminal command `plc`: the state directly, commands through the PLC thread (result in the log)
    ok, out = term(ws, "plc")
    check("terminal plc: the state", ok and "STOP, control source IEC 61131, autostart on" in out, out)
    ok, out = term(ws, "plc reset")
    check("terminal plc reset without warm|cold: refused as wrong arguments", not ok, out)
    mark = len(log)
    ok, out = term(ws, "plc source fieldbus")
    check("terminal plc source fieldbus: sent", ok and "sent to the PLC thread" in out, out)
    check("terminal plc source fieldbus: done (log)", wait_log("plc: terminal command done (STOP, control source fieldbus", 10, mark))
    check("control source fieldbus (status)", not status(ws)["iec"])
    check("fieldbus again: Modbus writes the conveyor", modbus_write(mb, CONVEYOR, 250) is None and holding(mb, CONVEYOR) == [250])
    mark = len(log)
    ok, out = term(ws, "plc autostart off")
    check("terminal plc autostart off: done (log)", ok and wait_log("autostart off)", 10, mark), out)
    check("autostart off (status)", not status(ws)["autostart"])
finally:
    for c in (ws, mb):
        if c is not None:
            c.close()
    stop.set()
    proc.kill()
    proc.wait()
    card_dir.cleanup()
    # also when the test ended with an exception: the firmware's side of it
    print("--- firmware log (plc, sd, errors):" if sys.exc_info()[0] is None else "--- firmware log (the test ended early):")
    for line in log if sys.exc_info()[0] is None else log[-60:]:
        if sys.exc_info()[0] is not None or "plc" in line or "sd:" in line or "ERR" in line:
            print("  " + line)
print("RESULT:", "PASS" if failures == 0 else f"FAIL ({failures})")
sys.exit(1 if failures else 0)
