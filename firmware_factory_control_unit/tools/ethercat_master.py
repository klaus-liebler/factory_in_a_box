#!/usr/bin/env python3
"""A minimal EtherCAT master for the FCU's EtherCAT slave (src/ethercat.cpp, rtoos rtos::ethercat).

Not a full master: it does on the wire what TwinCAT or SOEM do, datagram by datagram, and prints every
step - so a problem shows where it is:

  1. count the slaves (BRD), give the first one a station address (APWR 0x0010)
  2. ESC identity and state: type, revision, FMMUs, SyncManagers, ports, DL status (links,
     "PDI operational" = the configuration area was loaded from the emulated EEPROM), PDI type,
     AL status
  3. read the slave information (SII) through the EEPROM interface (0x0500-0x050F): the FCU's
     firmware answers every command from RAM (EEPROM emulation) - checks the configuration area's
     CRC, prints identity, strings and the general category
  4. AL state machine: INIT -> PRE-OP (mailbox SyncManagers from the SII) -> SAFE-OP (process data
     SyncManagers sized from the PDO assignment read over CoE) -> OP -> INIT; the slave's checks
     (a wrong SyncManager is refused with its AL status code) and BOOT refused
  5. CoE over the mailbox: SDO upload/download (the register map's object dictionary: inputs 6000h +
     Modbus address of the region, outputs 7000h + ..., docs/generated/modbus-registers.md), SDO
     information (the object list with names)
  6. process data in OP: FMMUs, cyclic LWR/LRD; an output (WS2812 pattern) through the RxPDO, read
     back over SDO; then no more cycles: the SyncManager watchdog must put the outputs to 0 (SAFE-OP,
     error 0x001B)

Transport: raw Ethernet frames (EtherType 0x88A4) through Npcap and scapy. Connect a PC network
port directly to the FCU's EtherCAT port A (port 0, "IN"); a straight cable is fine (the firmware sets
both PHYs to autonegotiation with Auto-MDIX, also port B, which revision 1 straps to fixed 100FD).

  pip install scapy           (Npcap must be installed - Windows)
  python tools/ethercat_master.py --list
  python tools/ethercat_master.py --iface "Realtek USB GbE" test
  python tools/ethercat_master.py --iface ... scan | sii | state op | watch | od
  python tools/ethercat_master.py --iface ... sdo read 0x1018 2 | sdo write 0x7064 1 3
  python tools/ethercat_master.py --iface ... blink      (WS2812 patterns over SDO, 7064h:01/02)
  python tools/ethercat_master.py --simulate scan|sii|state   (no hardware: a simulated ESC, no CoE)

The firmware's EtherCAT thread polls the ESC every 2 ms (revision 1) or wakes on its IRQ (revision 3).
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass

ETHERTYPE = 0x88A4

# datagram commands (ETG.1000.4)
APRD, APWR, FPRD, FPWR, BRD, BWR, LRD, LWR = 1, 2, 4, 5, 7, 8, 10, 11
LOGICAL_OUTPUTS, LOGICAL_INPUTS = 0x0001_0000, 0x0002_0000  # where this master maps the process data

# ESC registers (ETG.1000.4 / LAN9253 DS00003421 11.16)
REG = {
    "type": 0x0000, "revision": 0x0001, "build": 0x0002, "fmmus": 0x0004, "syncmanagers": 0x0005,
    "ram_kb": 0x0006, "ports": 0x0007, "features": 0x0008, "station": 0x0010, "dl_status": 0x0110,
    "al_control": 0x0120, "al_status": 0x0130, "al_status_code": 0x0134, "pdi_control": 0x0140,
    "pdi_config": 0x0150, "eeprom_config": 0x0500, "eeprom_pdi_access": 0x0501, "eeprom_status": 0x0502,
    "eeprom_address": 0x0504, "eeprom_data": 0x0508,
}
STATES = {"init": 1, "preop": 2, "boot": 3, "safeop": 4, "op": 8}
STATE_NAMES = {1: "INIT", 2: "PRE-OP", 3: "BOOT", 4: "SAFE-OP", 8: "OP"}
PDI_NAMES = {0x04: "digital I/O", 0x05: "Beckhoff SPI", 0x80: "SPI (LAN9252 compatible)",
             0x82: "SPI, EtherCAT Direct Mapped"}
STATION = 0x1001

# CoE (ETG.1000.6): mailbox type, services, SDO abort codes
MBX_COE, MBX_ERROR = 3, 0
COE_SDO_REQUEST, COE_SDO_RESPONSE, COE_SDO_INFO = 2, 3, 8
SDO_ABORTS = {0x05040001: "command not supported", 0x06010000: "unsupported access", 0x06010001: "write only",
              0x06010002: "read only", 0x06020000: "object does not exist", 0x06070012: "too long",
              0x06070013: "too short", 0x06090011: "subindex does not exist", 0x06090030: "value range",
              0x08000000: "general error", 0x08000021: "local control (the PLC owns it)",
              0x08000022: "not in this state"}
AL_CODES = {0x0011: "invalid state change", 0x0012: "unknown state", 0x0013: "bootstrap not supported",
            0x0016: "invalid mailbox configuration", 0x001B: "sync manager watchdog",
            0x001D: "invalid output configuration", 0x001E: "invalid input configuration"}
# the register map's objects (index = range base + the region's first Modbus address)
WS2812_OUTPUTS = 0x7000 + 100  # subindex 1: strip pattern (0 off, 1 red, ...), 2: single LED palette


class MasterError(Exception):
    pass


# ---- frames --------------------------------------------------------------------------------------
@dataclass
class Datagram:
    cmd: int
    adp: int
    ado: int
    data: bytes
    index: int = 0
    wkc: int = 0


def build_frame(src_mac: bytes, datagrams: list[Datagram]) -> bytes:
    body = b""
    for i, d in enumerate(datagrams):
        more = 0x8000 if i + 1 < len(datagrams) else 0
        body += struct.pack("<BBHHHH", d.cmd, d.index, d.adp & 0xFFFF, d.ado, len(d.data) | more, 0)
        body += d.data + struct.pack("<H", d.wkc)  # 0 from the master; a slave counts it up
    header = struct.pack("<H", len(body) | (1 << 12))  # length, type 1 (EtherCAT commands)
    frame = b"\xff" * 6 + src_mac + struct.pack(">H", ETHERTYPE) + header + body
    return frame + b"\x00" * max(0, 60 - len(frame))  # minimum frame (without FCS)


def parse_frame(frame: bytes) -> list[Datagram]:
    if len(frame) < 16 or struct.unpack(">H", frame[12:14])[0] != ETHERTYPE:
        return []
    (header,) = struct.unpack("<H", frame[14:16])
    length, pos, out = header & 0x7FF, 16, []
    end = 16 + length
    while pos + 10 <= end:
        cmd, idx, adp, ado, ln, _irq = struct.unpack("<BBHHHH", frame[pos:pos + 10])
        n = ln & 0x7FF
        data = frame[pos + 10:pos + 10 + n]
        (wkc,) = struct.unpack("<H", frame[pos + 10 + n:pos + 12 + n])
        out.append(Datagram(cmd, adp, ado, data, idx, wkc))
        pos += 12 + n
        if not ln & 0x8000:
            break
    return out


# ---- links ----------------------------------------------------------------------------------------
class PcapLink:
    """Raw Ethernet through scapy/Npcap. The frame comes back from the ESC with the locally
    administered bit set in the source address; Npcap also shows our own frame - ignored."""

    def __init__(self, iface: str):
        try:
            from scapy.all import conf, get_if_hwaddr  # noqa: PLC0415
        except ImportError as e:
            raise MasterError("scapy missing: pip install scapy (and install Npcap)") from e
        self.iface = resolve_iface(iface)
        self.mac = bytes.fromhex(get_if_hwaddr(self.iface).replace(":", ""))
        self.sock = conf.L2socket(iface=self.iface, filter=f"ether proto {ETHERTYPE:#x}", promisc=True)

    def transact(self, frame: bytes, timeout: float) -> bytes | None:
        self.sock.send(frame)
        deadline = time.monotonic() + timeout
        while (remain := deadline - time.monotonic()) > 0:
            if not self.sock.select([self.sock], remain):
                continue
            pkt = self.sock.recv()
            if pkt is None:
                continue
            raw = bytes(pkt)
            if raw[:len(frame)] == frame or raw[6:12] == self.mac:
                continue  # our own frame, seen on the way out
            return raw
        return None


def resolve_iface(name: str):
    from scapy.all import conf  # noqa: PLC0415
    for iface in conf.ifaces.values():
        if name in (iface.name, iface.description, getattr(iface, "network_name", None)) or name.lower() in iface.description.lower():
            return iface
    raise MasterError(f"no interface matches {name!r} (--list shows them)")


def list_ifaces() -> None:
    try:
        from scapy.all import conf  # noqa: PLC0415
    except ImportError:
        sys.exit("scapy missing: pip install scapy (and install Npcap)")
    for iface in conf.ifaces.values():
        print(f"  {iface.description:45s}  {iface.mac or '':17s}  {iface.network_name}")


class SimEsc:
    """A simulated LAN9253 with a slave behind it: EEPROM emulation from an SII image, AL state
    requests acknowledged without checks (BOOT refused), no mailbox. Checks the master's scan, SII and
    state sequences without hardware (the firmware's stack itself: rtoos tests/ethercat_test.cpp).
    Like the real one, AL control (0x0120) is a mailbox (ESC configuration 0x0141 bit 0 = 0,
    DS00003421 note 8): after a write the master cannot write it again (working counter 0) until the
    firmware has read it."""

    PDI_POLL = 0.05

    def __init__(self, pdi: int = 0x82):
        self.reg = bytearray(0x3000)  # registers 0x0000.., process RAM 0x1000.. (8 KB)
        self.reg[0x0000:0x0009] = bytes([0xC0, 0x01, 0x53, 0x92, 8, 8, 8, 0x0F, 0xFC])  # type, rev, build, ...
        self.reg[0x0140] = pdi
        self.sii = build_sii(pdi, 0)
        self.reg[0x0110:0x0112] = struct.pack("<H", 0x0001 | 0x0010 | 0x0200)  # PDI ok, port 0 link + comm
        self.reg[0x0130:0x0132] = struct.pack("<H", 1)
        self.reg[0x0502:0x0504] = struct.pack("<H", (1 << 5) | (1 << 6))  # emulation, 8-byte reads
        self.mac = b"\x00\x11\x22\x33\x44\x55"
        self.al_control_written: float | None = None  # time of the master's write the firmware has not read yet

    def u16(self, a): return struct.unpack("<H", self.reg[a:a + 2])[0]

    def transact(self, frame: bytes, timeout: float) -> bytes | None:
        self.firmware_cycle()
        out = []
        for d in parse_frame(frame):
            data, wkc = d.data, 0
            addressed = d.cmd in (BRD, BWR) or (d.cmd in (APRD, APWR) and d.adp == 0) or \
                (d.cmd in (FPRD, FPWR) and d.adp == self.u16(0x0010))
            if addressed:
                if d.cmd in (BRD, APRD, FPRD):
                    data, wkc = bytes(self.reg[d.ado:d.ado + len(d.data)]), 1
                else:
                    wkc = 1 if self.write(d.ado, d.data) else 0
            out.append(Datagram(d.cmd, (d.adp + 1) & 0xFFFF if d.cmd in (APRD, APWR) else d.adp, d.ado, data, d.index, wkc))
        reply = bytearray(build_frame(self.mac, out))
        reply[6] |= 0x02
        return bytes(reply)

    def write(self, a: int, data: bytes) -> bool:
        """False: not written (AL control still unread by the PDI)."""
        if a <= 0x0120 < a + len(data):
            if self.al_control_written is not None:
                return False
            self.al_control_written = time.monotonic()
        self.reg[a:a + len(data)] = data
        if a <= 0x0502 < a + len(data) and len(data) >= 2:  # EEPROM command
            cmd = (self.u16(0x0502) >> 8) & 7
            if cmd == 1:
                w = struct.unpack("<I", self.reg[0x0504:0x0508])[0]
                words = [self.sii[w + i] if w + i < len(self.sii) else 0xFFFF for i in range(4)]
                self.reg[0x0508:0x0510] = struct.pack("<4H", *words)
            self.reg[0x0502:0x0504] = struct.pack("<H", (1 << 5) | (1 << 6))  # done, not busy
        return True

    def firmware_cycle(self) -> None:
        self.serve_al()

    def serve_al(self) -> None:
        """serve_al() of the firmware, one I/O cycle after the master's write: reads AL control (the
        mailbox is free again) and acknowledges the state."""
        if self.al_control_written is None or time.monotonic() - self.al_control_written < self.PDI_POLL:
            return
        self.al_control_written = None
        want = self.reg[0x0120] & 0x0F
        if want == 3:
            self.reg[0x0130:0x0132] = struct.pack("<H", (self.reg[0x0130] & 0x0F) | 0x10)
            self.reg[0x0134:0x0136] = struct.pack("<H", 0x0011)
        elif want in STATE_NAMES:
            self.reg[0x0130:0x0132] = struct.pack("<H", want)


# ---- an SII image for the simulation (the firmware builds its own: src/ethercat_config.hpp) ----------
def config_crc(words: list[int]) -> int:
    crc = 0xFF
    for i in range(14):
        crc ^= (words[i // 2] >> (8 * (i % 2))) & 0xFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_sii(pdi: int, pdi_cfg: int) -> list[int]:
    w = [0] * 128
    w[0], w[1] = pdi, pdi_cfg
    w[7] = config_crc(w)
    w[0x0A], w[0x0C], w[0x3E], w[0x3F] = 0x9253, 1, 1, 1
    name = b"FCU EtherCAT test"
    data = bytes([1, len(name)]) + name
    data += b"\x00" * (len(data) % 2)
    pos = 0x40
    w[pos], w[pos + 1] = 10, len(data) // 2
    pos += 2
    for i in range(0, len(data), 2):
        w[pos] = data[i] | (data[i + 1] << 8)
        pos += 1
    w[pos], w[pos + 1], w[pos + 2], w[pos + 3] = 30, 16, 0x0001, 0x0101
    w[pos + 18] = 0xFFFF
    return w


# ---- the master ------------------------------------------------------------------------------------
def sii_end(w: list[int]) -> int | None:
    """The word index of the categories' end marker, None while not read that far."""
    pos = 0x40
    while pos + 1 < len(w):
        if w[pos] == 0xFFFF:
            return pos
        pos += 2 + w[pos + 1]
    return None


def sii_sync_managers(w: list[int]) -> list[tuple[int, int, int]]:
    """(start, length, control) of each SyncManager of the SII's category 41."""
    pos = 0x40
    while pos + 1 < len(w) and w[pos] != 0xFFFF:
        if w[pos] == 41:
            body = b"".join(struct.pack("<H", x) for x in w[pos + 2:pos + 2 + w[pos + 1]])
            return [struct.unpack("<HHB", body[8 * n:8 * n + 5]) for n in range(len(body) // 8)]
        pos += 2 + w[pos + 1]
    return []


class Master:
    def __init__(self, link, verbose: bool = False):
        self.link, self.verbose, self.index = link, verbose, 0
        self.sms: list[tuple[int, int, int]] = []
        self.mbx_counter = 0

    def _do(self, cmd: int, adp: int, ado: int, data: bytes, timeout: float = 0.25) -> Datagram:
        # The ESC answers within microseconds, but a USB adapter and Npcap now and then hold a received
        # frame back for 100 ms and more: several attempts with a generous timeout; a late answer to
        # an earlier attempt has the same index and counts, one to an older datagram is skipped.
        self.index = (self.index + 1) & 0xFF
        frame = build_frame(self.link.mac, [Datagram(cmd, adp, ado, data, self.index)])
        for _ in range(5):
            reply = self.link.transact(frame, timeout)
            if reply is None:
                continue
            for d in parse_frame(reply):
                if d.index == self.index:
                    if self.verbose:
                        print(f"    cmd {cmd} adp {adp:#06x} ado {ado:#06x} -> wkc {d.wkc} data {d.data.hex()}")
                    return d
        raise MasterError("no frame came back - cable on port A (IN)? interface? (the PC sees no EtherCAT device)")

    def read(self, ado: int, n: int) -> bytes:
        d = self._do(FPRD, STATION, ado, bytes(n))
        if d.wkc != 1:
            raise MasterError(f"FPRD {ado:#06x}: working counter {d.wkc} (expected 1)")
        return d.data

    def write(self, ado: int, data: bytes) -> None:
        d = self._do(FPWR, STATION, ado, data)
        if d.wkc != 1:
            raise MasterError(f"FPWR {ado:#06x}: working counter {d.wkc} (expected 1)")

    def r8(self, ado): return self.read(ado, 1)[0]
    def r16(self, ado): return struct.unpack("<H", self.read(ado, 2))[0]

    # 1. slaves, station address
    def init_bus(self) -> int:
        count = self._do(BRD, 0, REG["type"], bytes(2)).wkc
        print(f"slaves on the bus: {count}")
        if count == 0:
            raise MasterError("the frame came back, but no slave answered (working counter 0)")
        d = self._do(APWR, 0, REG["station"], struct.pack("<H", STATION))
        if d.wkc != 1:
            raise MasterError("setting the station address failed")
        print(f"first slave: station address {STATION:#06x}")
        return count

    # 2. identity, links, state
    def describe(self) -> dict:
        info = self.read(0x0000, 10)
        esc_type, rev, build, fmmus, sms, ram, ports, features = struct.unpack("<BBHBBBBH", info)
        dl = self.r16(REG["dl_status"])
        pdi = self.r8(REG["pdi_control"])
        al = self.r16(REG["al_status"])
        print(f"ESC type {esc_type:#04x} rev {rev} build {build:#06x}: {fmmus} FMMUs, {sms} SyncManagers, "
              f"{ram} KB RAM, ports {ports:#04x}, features {features:#06x}")
        print(f"DL status {dl:#06x}: PDI {'operational (configuration loaded)' if dl & 1 else 'NOT operational'}, "
              f"port 0 link {'up' if dl & 0x10 else 'down'}, port 1 link {'up' if dl & 0x20 else 'down'}")
        print(f"PDI control {pdi:#04x}: {PDI_NAMES.get(pdi, 'other')}")
        print(f"AL status {al:#06x}: {STATE_NAMES.get(al & 0x0F, '?')}{', ERROR' if al & 0x10 else ''}")
        return {"dl": dl, "pdi": pdi, "al": al}

    # 3. SII through the EEPROM interface
    def eeprom_read(self, word: int, timeout: float = 1.0) -> bytes:
        deadline = time.monotonic() + timeout
        while self.r16(REG["eeprom_status"]) & 0x8000:
            if time.monotonic() > deadline:
                raise MasterError("EEPROM interface stays busy")
        self.write(REG["eeprom_address"], struct.pack("<I", word))
        self.write(REG["eeprom_status"], struct.pack("<H", 0x0100))  # command: read
        while True:
            status = self.r16(REG["eeprom_status"])
            if not status & 0x8000:
                break
            if time.monotonic() > deadline:
                raise MasterError(f"EEPROM read of word {word:#x}: no answer (the firmware serves it every 50 ms - "
                                  "is it running, did ethercat::setup() find the LAN9253?)")
            time.sleep(0.005)
        if status & 0x6000:
            raise MasterError(f"EEPROM read of word {word:#x}: error bits {status:#06x}")
        return self.read(REG["eeprom_data"], 8 if status & 0x40 else 4)

    def read_sii(self, words: int = 512) -> list[int]:
        """Up to the categories' end marker (or `words`); the SyncManager category into self.sms."""
        self.write(REG["eeprom_config"], b"\x02")  # force: the EEPROM to EtherCAT (SOEM's ec_eeprom2master)
        self.write(REG["eeprom_config"], b"\x00")
        out: list[int] = []
        while len(out) < words:
            data = self.eeprom_read(len(out))
            out += list(struct.unpack(f"<{len(data) // 2}H", data))
            if len(out) > 0x40 and sii_end(out) is not None:
                break
        self.sms = sii_sync_managers(out)
        return out[:words]

    @staticmethod
    def print_sii(w: list[int]) -> bool:
        crc_ok = config_crc(w) == (w[7] & 0xFF)
        print(f"SII config area: PDI control {w[0] & 0xFF:#04x}, PDI config {w[1] & 0xFF:#04x}, "
              f"CRC {w[7] & 0xFF:#04x} ({'ok' if crc_ok else 'WRONG, expected %#04x' % config_crc(w)})")
        print(f"SII identity: vendor {w[8] | w[9] << 16:#010x}, product {w[10] | w[11] << 16:#010x}, "
              f"revision {w[12] | w[13] << 16}, serial {w[14] | w[15] << 16:#010x}, size {(w[0x3E] + 1)} Kbit, version {w[0x3F]}")
        if w[0x19]:
            print(f"SII mailbox: out {w[0x18]:#06x} ({w[0x19]} bytes), in {w[0x1A]:#06x} ({w[0x1B]} bytes), protocols {w[0x1C]:#06x}")
        pos = 0x40
        while pos + 1 < len(w) and w[pos] != 0xFFFF:
            cat, size = w[pos], w[pos + 1]
            body = b"".join(struct.pack("<H", x) for x in w[pos + 2:pos + 2 + size])
            if cat == 10:
                strings, p = [], 1
                for _ in range(body[0] if body else 0):
                    n = body[p]
                    strings.append(body[p + 1:p + 1 + n].decode("ascii", "replace"))
                    p += 1 + n
                print(f"SII strings: {strings}")
            elif cat == 30:
                print(f"SII general: group {body[0]}, image {body[1]}, order {body[2]}, name {body[3]}, CoE details {body[5]:#04x}")
            elif cat == 41:
                for n in range(len(body) // 8):
                    start, length, control, _, enable, kind = struct.unpack("<HHBBBB", body[8 * n:8 * n + 8])
                    print(f"SII SyncManager {n}: start {start:#06x}, length {length}, control {control:#04x}, "
                          f"enable {enable}, type {['unused', 'mailbox out', 'mailbox in', 'outputs', 'inputs'][kind] if kind < 5 else kind}")
            else:
                print(f"SII category {cat}, {size} words")
            pos += 2 + size
        return crc_ok

    # 4. AL state machine
    def write_al_control(self, value: int, timeout: float = 1.0) -> None:
        """AL control is a mailbox (Device Emulation off): working counter 0 as long as the firmware
        has not read the previous request (it polls every 50 ms) - write again until it takes it."""
        deadline = time.monotonic() + timeout
        while True:
            d = self._do(FPWR, STATION, REG["al_control"], struct.pack("<H", value))
            if d.wkc == 1:
                return
            if time.monotonic() > deadline:
                raise MasterError(f"AL control {value:#06x}: working counter {d.wkc} for {timeout} s - "
                                  "the firmware does not read AL control (serve_al())")
            time.sleep(0.01)

    def request_state(self, name: str, timeout: float = 2.0) -> bool:
        want = STATES[name]
        self.write_al_control(want)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            al = self.r16(REG["al_status"])
            if al & 0x10:
                code = self.r16(REG["al_status_code"])
                print(f"  -> {STATE_NAMES.get(want)}: refused, AL status {al:#06x}, code {code:#06x}")
                self.write_al_control((al & 0x0F) | 0x10)  # acknowledge the error
                return False
            if al & 0x0F == want:
                print(f"  -> {STATE_NAMES[want]}: ok")
                return True
            time.sleep(0.01)
        print(f"  -> {STATE_NAMES[want]}: no answer within {timeout} s (AL status still {self.r16(REG['al_status']):#06x})")
        return False

    # 5. SyncManagers as the SII says (the master's part before PRE-OP / SAFE-OP)
    def configure_sm(self, n: int, length: int | None = None, enable: bool = True) -> None:
        start, sii_length, control = self.sms[n]
        length = sii_length if length is None else length
        self.write(0x0800 + 8 * n + 6, b"\x00")  # disable: start/length/control writable
        self.write(0x0800 + 8 * n, struct.pack("<HHB", start, length, control))
        self.write(0x0800 + 8 * n + 6, b"\x01" if enable and length > 0 else b"\x00")

    def configure_mailbox(self) -> None:
        self.configure_sm(0)
        self.configure_sm(1)
        self.mbx_counter = 0

    # 6. mailbox (ETG.1000.4 5.6): SM0 master -> slave, SM1 slave -> master
    def mailbox(self, mtype: int, data: bytes, timeout: float = 1.0) -> tuple[int, bytes]:
        out_start, out_size, _ = self.sms[0]
        in_start, in_size, _ = self.sms[1]
        if len(data) > out_size - 6:
            raise MasterError(f"mailbox message of {len(data)} bytes: the mailbox holds {out_size - 6}")
        deadline = time.monotonic() + timeout
        while self.r8(0x0805) & 0x08:  # SM0 still full: the slave has not read the last one
            if time.monotonic() > deadline:
                raise MasterError("mailbox: the slave does not read SM0")
            time.sleep(0.002)
        self.mbx_counter = self.mbx_counter % 7 + 1
        frame = struct.pack("<HHBB", len(data), 0, 0, mtype | self.mbx_counter << 4) + data
        self.write(out_start, frame + bytes(out_size - len(frame)))  # the whole buffer: its last byte marks it full
        return self.mailbox_receive(timeout)

    def mailbox_receive(self, timeout: float = 1.0) -> tuple[int, bytes]:
        in_start, in_size, _ = self.sms[1]
        deadline = time.monotonic() + timeout
        while not self.r8(0x080D) & 0x08:  # SM1 full: the slave's response
            if time.monotonic() > deadline:
                raise MasterError(f"mailbox: no response within {timeout} s")
            time.sleep(0.002)
        b = self.read(in_start, in_size)  # the whole buffer: reading its last byte frees it
        length, _, _, kind = struct.unpack("<HHBB", b[:6])
        mtype, body = kind & 0x0F, b[6:6 + length]
        if mtype == MBX_ERROR:
            detail = struct.unpack("<H", body[2:4])[0] if len(body) >= 4 else 0
            raise MasterError(f"mailbox error {detail:#06x}")
        return mtype, body

    # 7. CoE SDO (ETG.1000.6 5.6.2): expedited and normal transfers
    def _sdo(self, cmd: int, index: int, sub: int, data: bytes = bytes(4), more: bytes = b"") -> bytes:
        req = struct.pack("<HBHB", COE_SDO_REQUEST << 12, cmd, index, sub) + data + more
        mtype, r = self.mailbox(MBX_COE, req)
        if mtype != MBX_COE or len(r) < 10:
            raise MasterError(f"SDO {index:#06x}:{sub}: unexpected response {r.hex()}")
        if r[2] == 0x80:
            code = struct.unpack("<I", r[6:10])[0]
            raise MasterError(f"SDO {index:#06x}:{sub}: abort {code:#010x} ({SDO_ABORTS.get(code, '?')})")
        return r

    def sdo_read(self, index: int, sub: int) -> bytes:
        r = self._sdo(0x40, index, sub)
        if r[2] & 0x02:  # expedited
            n = 4 - ((r[2] >> 2) & 3) if r[2] & 0x01 else 4
            return r[6:6 + n]
        n = struct.unpack("<I", r[6:10])[0]
        return r[10:10 + n]

    def sdo_write(self, index: int, sub: int, value: bytes) -> None:
        if len(value) <= 4:
            cmd = 0x23 | ((4 - len(value)) << 2)  # expedited, size indicated
            self._sdo(cmd, index, sub, value + bytes(4 - len(value)))
        else:
            self._sdo(0x21, index, sub, struct.pack("<I", len(value)), value)

    # 8. SDO information (ETG.1000.6 5.6.3): a response may come in fragments
    def sdo_info(self, opcode: int, data: bytes) -> bytes:
        mtype, r = self.mailbox(MBX_COE, struct.pack("<HBBH", COE_SDO_INFO << 12, opcode, 0, 0) + data)
        body = b""
        while True:
            if r[2] & 0x7F == 7:
                code = struct.unpack("<I", r[6:10])[0]
                raise MasterError(f"SDO info: error {code:#010x} ({SDO_ABORTS.get(code, '?')})")
            body += r[6:]
            if not r[2] & 0x80:  # complete
                return body
            _, r = self.mailbox_receive()

    def object_list(self) -> list[int]:
        body = self.sdo_info(1, struct.pack("<H", 1))
        return list(struct.unpack(f"<{(len(body) - 2) // 2}H", body[2:]))

    def object_name(self, index: int) -> tuple[str, int]:
        body = self.sdo_info(3, struct.pack("<H", index))
        return body[6:].decode("ascii", "replace"), body[4]

    def mapping(self, assign: int) -> list[tuple[int, int, int, int]]:
        """(index, subindex, bits, byte offset) of every entry of the assigned PDOs, in image order."""
        out, offset = [], 0
        for k in range(1, self.sdo_read(assign, 0)[0] + 1):
            pdo = struct.unpack("<H", self.sdo_read(assign, k))[0]
            for e in range(1, self.sdo_read(pdo, 0)[0] + 1):
                v = struct.unpack("<I", self.sdo_read(pdo, e))[0]
                out.append((v >> 16, (v >> 8) & 0xFF, v & 0xFF, offset))
                offset += (v & 0xFF) // 8
        return out

    # 9. process data: FMMUs onto SM2/SM3, cyclic logical reads/writes
    def configure_fmmus(self, out_bytes: int, in_bytes: int) -> None:
        for n, (logical, length, physical, kind) in enumerate([(LOGICAL_OUTPUTS, out_bytes, self.sms[2][0], 2),
                                                               (LOGICAL_INPUTS, in_bytes, self.sms[3][0], 1)]):
            self.write(0x0600 + 16 * n, struct.pack("<IHBBHBBB3x", logical, length, 0, 7, physical, 0, kind, 1 if length else 0))

    def exchange(self, outputs: bytes, in_bytes: int) -> bytes:
        if outputs:
            d = self._do(LWR, LOGICAL_OUTPUTS & 0xFFFF, LOGICAL_OUTPUTS >> 16, outputs)
            if d.wkc != 1:
                raise MasterError(f"LWR outputs: working counter {d.wkc} (FMMU 0 / SM2?)")
        if not in_bytes:
            return b""
        d = self._do(LRD, LOGICAL_INPUTS & 0xFFFF, LOGICAL_INPUTS >> 16, bytes(in_bytes))
        if d.wkc != 1:
            raise MasterError(f"LRD inputs: working counter {d.wkc} (FMMU 1 / SM3?)")
        return d.data

    def assigned_bytes(self) -> tuple[int, int]:
        """RxPDO and TxPDO bytes of the assignment (1C12h/1C13h) and the mappings."""
        sizes = []
        for assign in (0x1C12, 0x1C13):
            bits = 0
            for k in range(1, self.sdo_read(assign, 0)[0] + 1):
                pdo = struct.unpack("<H", self.sdo_read(assign, k))[0]
                for e in range(1, self.sdo_read(pdo, 0)[0] + 1):
                    bits += struct.unpack("<I", self.sdo_read(pdo, e))[0] & 0xFF
            sizes.append((bits + 7) // 8)
        return sizes[0], sizes[1]


# ---- commands ---------------------------------------------------------------------------------------
def process_data_test(m: Master, out_bytes: int, in_bytes: int) -> bool:
    """SAFE-OP -> OP with cyclic process data; an output through the RxPDO; the watchdog."""
    ok = True
    rx = m.mapping(0x1C12)
    pattern = next((e for e in rx if e[0] == WS2812_OUTPUTS and e[1] == 1), None)
    m.configure_fmmus(out_bytes, in_bytes)
    outputs = bytearray(out_bytes)
    if not m.request_state("safeop"):
        return False
    m.exchange(bytes(outputs), in_bytes)  # valid outputs before OP
    m.write_al_control(STATES["op"])
    deadline, state = time.monotonic() + 2.0, 0
    while time.monotonic() < deadline and state & 0x1F != 8:
        m.exchange(bytes(outputs), in_bytes)  # keep the watchdog fed while the slave takes OP
        state = m.r16(REG["al_status"])
        time.sleep(0.01)
    print(f"  -> OP: {'ok' if state & 0x1F == 8 else 'FAILED, AL status %#06x' % state}")
    if state & 0x1F != 8:
        return False
    inputs = b""
    if pattern is not None:
        struct.pack_into("<H", outputs, pattern[3], 3)  # strip pattern 3: green
    for _ in range(30):  # 300 ms of cycles
        inputs = m.exchange(bytes(outputs), in_bytes)
        time.sleep(0.01)
    print(f"  process data: {out_bytes} bytes out, {len(inputs)} bytes in, inputs {inputs[:16].hex()}...")
    if pattern is not None:
        got = struct.unpack("<H", m.sdo_read(WS2812_OUTPUTS, 1))[0]
        print(f"  WS2812 pattern through the RxPDO: {got} (expected 3){'' if got == 3 else ' - FAIL'}")
        ok = ok and got == 3
    print("  no more cycles: the watchdog must stop the outputs")
    time.sleep(0.5)  # default watchdog 100 ms
    al, code = m.r16(REG["al_status"]), m.r16(REG["al_status_code"])
    wd_ok = al == (4 | 0x10) and code == 0x001B
    print(f"  -> AL status {al:#06x}, code {code:#06x}{'' if wd_ok else ' - FAIL: expected SAFE-OP + error, 0x001B'}")
    if pattern is not None:
        got = struct.unpack("<H", m.sdo_read(WS2812_OUTPUTS, 1))[0]
        print(f"  WS2812 pattern after the watchdog: {got} (expected 0){'' if got == 0 else ' - FAIL'}")
        wd_ok = wd_ok and got == 0
    m.write_al_control(4 | 0x10)  # acknowledge
    time.sleep(0.05)
    return ok and wd_ok


def expect_refused(m: Master, state: str, code: int) -> bool:
    """A request that the slave must refuse with `code` (then acknowledged)."""
    m.write_al_control(STATES[state])
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        al = m.r16(REG["al_status"])
        if al & 0x10:
            got = m.r16(REG["al_status_code"])
            m.write_al_control((al & 0x0F) | 0x10)  # acknowledge
            time.sleep(0.05)
            print(f"  -> refused with {got:#06x} ({AL_CODES.get(got, '?')})" + ("" if got == code else f" - expected {code:#06x}"))
            return got == code
        if al & 0x0F == STATES[state]:
            print(f"  -> FAIL: {state} accepted")
            return False
        time.sleep(0.01)
    print("  -> FAIL: no answer")
    return False


def run(args) -> int:
    link = SimEsc() if args.simulate else PcapLink(args.iface)
    m = Master(link, args.verbose)
    try:
        m.init_bus()
        info = m.describe()
        if args.command == "scan":
            return 0
        if args.command == "sii":
            return 0 if m.print_sii(m.read_sii()) else 1
        if args.command == "state":
            return 0 if m.request_state(args.state) else 1
        if args.command in ("sdo", "od", "blink"):
            m.read_sii()
            m.configure_mailbox()
            if not m.request_state("preop"):
                return 1
        if args.command == "sdo":
            if args.op == "write":
                m.sdo_write(args.index, args.sub, args.value.to_bytes(args.size, "little", signed=args.value < 0))
                print(f"{args.index:#06x}:{args.sub} := {args.value}")
            else:
                v = m.sdo_read(args.index, args.sub)
                as_int = int.from_bytes(v, "little") if len(v) <= 4 else None
                text = v.decode("ascii", "replace") if v and all(32 <= c < 127 for c in v) else ""
                print(f"{args.index:#06x}:{args.sub} = {v.hex()}" + (f" ({as_int})" if as_int is not None else "") + (f" \"{text}\"" if text else ""))
            return 0
        if args.command == "od":
            for index in m.object_list():
                name, max_sub = m.object_name(index)
                print(f"{index:#06x}  {name}  (subindex 0..{max_sub})")
            return 0
        if args.command == "blink":
            # strip (CH1) pattern 1 = red, single LED (CH2) palette 1 = red; 0 = off
            on = False
            try:
                n = 0
                while args.cycles == 0 or n < 2 * args.cycles:
                    on = not on
                    m.sdo_write(WS2812_OUTPUTS, 1, struct.pack("<H", args.pattern if on else 0))
                    m.sdo_write(WS2812_OUTPUTS, 2, struct.pack("<H", args.colour if on else 0))
                    print(time.strftime("%H:%M:%S"), "LEDs", "on" if on else "off", flush=True)
                    n += 1
                    time.sleep(args.interval)
            except KeyboardInterrupt:
                pass
            m.sdo_write(WS2812_OUTPUTS, 1, b"\x00\x00")
            m.sdo_write(WS2812_OUTPUTS, 2, b"\x00\x00")
            print("LEDs off")
            return 0
        if args.command == "watch":
            last = None
            while True:
                now = (m.r16(REG["dl_status"]), m.r16(REG["al_status"]))
                if now != last:
                    print(time.strftime("%H:%M:%S"), f"DL {now[0]:#06x} AL {now[1]:#06x}")
                    last = now
                time.sleep(0.2)
        # test: everything, end to end
        ok = True
        if not info["dl"] & 1:
            print("FAIL: PDI not operational - the ESC has not loaded its configuration from the emulated EEPROM")
            ok = False
        sii = m.read_sii()
        if not m.print_sii(sii):
            ok = False
        if sii[0] & 0xFF != info["pdi"]:
            print(f"FAIL: the SII's PDI type {sii[0] & 0xFF:#04x} is not the ESC's {info['pdi']:#04x}")
            ok = False
        print("AL state machine:")
        ok = m.request_state("init") and ok
        if args.simulate:  # no mailbox in the simulation: states only
            for state in ("preop", "safeop", "op", "init"):
                ok = m.request_state(state) and ok
        else:
            print("  PRE-OP without the mailbox SyncManagers: must be refused (0x0016)")
            for n in (0, 1):
                m.configure_sm(n, enable=False)
            ok = expect_refused(m, "preop", 0x0016) and ok
            m.configure_mailbox()
            ok = m.request_state("preop") and ok
            print("CoE:")
            ident = [struct.unpack("<I", m.sdo_read(0x1018, k))[0] for k in (1, 2, 3, 4)]
            print(f"  identity 1018h: vendor {ident[0]:#010x}, product {ident[1]:#010x}, revision {ident[2]}, serial {ident[3]:#010x}")
            if ident[1] != (sii[10] | sii[11] << 16) or ident[2] != (sii[12] | sii[13] << 16):
                print("  FAIL: identity over CoE is not the SII's")
                ok = False
            print(f"  device name {m.sdo_read(0x1008, 0).decode()!r}, hardware {m.sdo_read(0x1009, 0).decode()!r}, "
                  f"firmware {m.sdo_read(0x100A, 0).decode()!r}")
            objects = m.object_list()
            print(f"  SDO information: {len(objects)} objects")
            try:
                m.sdo_read(0x5555, 0)
                print("  FAIL: an object that does not exist was read")
                ok = False
            except MasterError as e:
                print(f"  object 5555h: {e} - as expected")
            out_bytes, in_bytes = m.assigned_bytes()
            print(f"  PDO assignment: {out_bytes} bytes outputs (SM2), {in_bytes} bytes inputs (SM3)")
            print("  SAFE-OP with a wrong SM2 length: must be refused (0x001D)")
            m.configure_sm(2, out_bytes + 2)
            m.configure_sm(3, in_bytes)
            ok = expect_refused(m, "safeop", 0x001D) and ok
            m.configure_sm(2, out_bytes)
            ok = process_data_test(m, out_bytes, in_bytes) and ok
            ok = m.request_state("init") and ok
        refused = not m.request_state("boot")
        print(f"  BOOT {'refused as expected' if refused else 'accepted - the firmware should refuse it'}")
        ok = ok and refused
        print("RESULT:", "PASS" if ok else "FAIL")
        return 0 if ok else 1
    except MasterError as e:
        print("ERROR:", e)
        return 2


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", help="network interface (name or part of its description; --list)")
    ap.add_argument("--list", action="store_true", help="list the network interfaces")
    ap.add_argument("--simulate", action="store_true", help="a simulated ESC instead of a network interface")
    ap.add_argument("-v", "--verbose", action="store_true", help="print every datagram")
    sub = ap.add_subparsers(dest="command")
    sub.add_parser("scan", help="slaves, ESC identity, links, PDI, AL state")
    sub.add_parser("sii", help="read and decode the SII (emulated EEPROM)")
    st = sub.add_parser("state", help="request an AL state")
    st.add_argument("state", choices=list(STATES))
    sub.add_parser("watch", help="print DL and AL status changes (Ctrl+C ends)")
    sd = sub.add_parser("sdo", help="CoE SDO read/write (goes to PRE-OP; the register map: 6000h/7000h + Modbus address)")
    sd_ops = sd.add_subparsers(dest="op", required=True)
    r = sd_ops.add_parser("read", help="SDO upload")
    r.add_argument("index", type=lambda s: int(s, 0))
    r.add_argument("sub", type=lambda s: int(s, 0))
    w = sd_ops.add_parser("write", help="SDO download")
    w.add_argument("index", type=lambda s: int(s, 0))
    w.add_argument("sub", type=lambda s: int(s, 0))
    w.add_argument("value", type=lambda s: int(s, 0))
    w.add_argument("--size", type=int, default=2, choices=(1, 2, 4), help="bytes (default 2: a register)")
    sub.add_parser("od", help="the object dictionary over SDO information (index, name)")
    bl = sub.add_parser("blink", help="RGB LEDs on/off over CoE (7064h:01/02, the WS2812 patterns; Ctrl+C ends, LEDs off)")
    bl.add_argument("--interval", type=float, default=1.0, help="seconds per on/off phase (default 1)")
    bl.add_argument("--cycles", type=int, default=0, help="on/off cycles, 0 = until Ctrl+C")
    bl.add_argument("--pattern", type=int, default=1, help="strip pattern when on (1 red, 2 yellow, 3 green, 4 blue/white)")
    bl.add_argument("--colour", type=int, default=1, help="single LED palette index when on (1 red, 2 green, 3 blue, ...)")
    sub.add_parser("test", help="end to end: scan, SII, AL states with the slave's checks, CoE identity/info/PDO assignment, process data in OP, watchdog, BOOT refused")
    args = ap.parse_args()
    if args.list:
        list_ifaces()
        return 0
    if not args.simulate and not args.iface:
        ap.error("--iface (or --simulate) is required")
    args.command = args.command or "test"
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
