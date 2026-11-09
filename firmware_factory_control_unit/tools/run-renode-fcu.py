#!/usr/bin/env python3
"""Runs the FCU firmware in Renode on the TAP network, for trying it without hardware: the web UI is
at https://192.168.77.2/ (the PLC page: https://192.168.77.2/plc), Modbus TCP and OPC UA on the same
address. DHCP, DNS and SNTP for the device come from this script (rtoos/emulation/net_test.py), the
TAP adapter "rtoos-tap" from rtoos/emulation/renode/setup-tap.ps1. Stop with Ctrl+C.

  cmake --build --preset Release-Rev3        (the firmware embeds web/dist: npm run build first)
  python tools/run-renode-fcu.py [--revision 2] [--seconds N] [--quiet]

The flash of the emulated FCU (and with it the PLC program) lives as long as this script runs.
"""
import argparse
import re
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--revision", type=int, choices=[2, 3], default=3)
ap.add_argument("--elf", type=Path)
ap.add_argument("--renode", default="C:/opt/renode/renode.exe")
ap.add_argument("--seconds", type=float, default=0, help="stop after this time (0: until Ctrl+C)")
ap.add_argument("--quiet", action="store_true", help="do not echo the firmware's console")
args = ap.parse_args()
rtoos = ROOT / "rtoos"
sys.path.insert(0, str(rtoos / "emulation"))
import net_test  # noqa: E402

elf = args.elf or ROOT / f"build/{'Release-Rev3' if args.revision == 3 else 'Release'}/firmware_factory_control_unit.elf"
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
deadline = time.monotonic() + 60
while True:
    try:
        con = socket.create_connection(("127.0.0.1", port), timeout=1)
        break
    except OSError:
        if time.monotonic() > deadline or proc.poll() is not None:
            proc.kill()
            sys.exit("Renode did not start")
        time.sleep(0.2)
ready = threading.Event()


def pump():
    buf = b""
    while True:
        try:
            d = con.recv(4096)
        except OSError:
            return
        if not d:
            return
        buf += d
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode(errors="replace").rstrip("\r")
            if "dhcp: bound" in text:
                ready.set()
            if not args.quiet:
                print(text, flush=True)


threading.Thread(target=pump, daemon=True).start()
print(f"FCU revision {args.revision} in Renode ({elf.name}); waiting for the network ...", flush=True)
if ready.wait(120):
    print(f"ready: https://{net_test.DEVICE_IP}/plc", flush=True)
else:
    print("no DHCP lease within 120 s (is the TAP adapter up?)", flush=True)
try:
    end = time.monotonic() + args.seconds if args.seconds > 0 else None
    while proc.poll() is None and (end is None or time.monotonic() < end):
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
finally:
    stop.set()
    proc.kill()
    proc.wait()
