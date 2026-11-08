#!/usr/bin/env python3
"""USB of the FCU firmware (hardware revision 3, STM32H743 USB2_OTG_FS) in Renode, without hardware:
rtoos' emulation/usb_test.py with its scripted USB host enumerates the composite device (CDC-NCM +
CDC-ACM), configures it and sends a Modbus RTU request over the CDC-ACM interface; the answer must
be valid (unit 1, function 3, CRC).

  cmake --build --preset Release-Rev3
  python tools/test-usb-renode.py [--elf build/Release-Rev3/firmware_factory_control_unit.elf]

The chip UID comes from generated/device_ids.hh (the firmware's identity check), USB_VSENSE (PC0)
is driven high (the firmware connects USB only while VBUS is there). Exit code 0 = pass.
"""
import argparse
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--elf", type=Path, default=ROOT / "build/Release-Rev3/firmware_factory_control_unit.elf")
args = ap.parse_args()
ids = (ROOT / "generated/device_ids.hh").read_text(encoding="utf-8")
uid = [re.search(rf"DEVICE_CHIP_UID_W{i} = 0x([0-9A-Fa-f]+)", ids).group(1) for i in range(3)]
sys.exit(subprocess.call([sys.executable, "-I", str(ROOT / "rtoos/emulation/usb_test.py"), str(args.elf), "--mode", "modbus",
                          "--uid", *uid, "--monitor", "sysbus.gpioPortC OnGPIO 0 true"]))
