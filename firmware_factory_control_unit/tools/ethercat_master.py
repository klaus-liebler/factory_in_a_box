#!/usr/bin/env python3
"""A minimal EtherCAT master for the FCU's LAN9253 bring-up test (src/ethercat_test.cpp).

Not a full master (no mailbox, no process data): it does on the wire what TwinCAT or SOEM do first,
datagram by datagram, and prints every step - so a bring-up problem shows where it is:

  1. count the slaves (BRD), give the first one a station address (APWR 0x0010)
  2. ESC identity and state: type, revision, FMMUs, SyncManagers, ports, DL status (links,
     "PDI operational" = the configuration area was loaded from the emulated EEPROM), PDI type,
     AL status
  3. read the slave information (SII) through the EEPROM interface (0x0500-0x050F): the FCU's
     firmware answers every command from RAM (EEPROM emulation) - checks the configuration area's
     CRC, prints identity, strings and the general category
  4. AL state machine: INIT -> PRE-OP -> SAFE-OP -> OP -> INIT, each acknowledged by the firmware

Transport: raw Ethernet frames (EtherType 0x88A4) through Npcap and scapy. Connect a PC network
port directly to the FCU's EtherCAT port A (port 0, "IN"); a straight cable is fine (the firmware sets
both PHYs to autonegotiation with Auto-MDIX, also port B, which revision 1 straps to fixed 100FD).

  pip install scapy           (Npcap must be installed - Windows)
  python tools/ethercat_master.py --list
  python tools/ethercat_master.py --iface "Realtek USB GbE" test
  python tools/ethercat_master.py --iface ... scan | sii | state op | watch
  python tools/ethercat_master.py --simulate test      (no hardware: a simulated ESC, see SimEsc)

The firmware polls the ESC every I/O cycle (50 ms), so EEPROM reads and state changes take up to
one cycle each; the timeouts below allow for that.
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass

ETHERTYPE = 0x88A4

# datagram commands (ETG.1000.4)
APRD, APWR, FPRD, FPWR, BRD, BWR = 1, 2, 4, 5, 7, 8

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

# register access over EtherCAT (src/ethercat_registers.hpp): request/response areas in process RAM
RA_REQUEST, RA_RESPONSE, RA_HEADER, RA_MAX_COUNT = 0x1000, 0x1100, 8, 64
READ_HOLDING, READ_INPUT, WRITE_HOLDING = 0x03, 0x04, 0x10  # Modbus function codes
MODBUS_EXCEPTIONS = {1: "illegal function", 2: "illegal data address - or refused: the PLC owns the outputs",
                     3: "illegal data value", 4: "server device failure"}
# registers of docs/generated/modbus-registers.md used here
WS2812_CH1_PATTERN, WS2812_CH2_PATTERN = 100, 101  # holding: strip pattern (0 off, 1 red, ...), single LED palette
FW_VERSION_MAJOR = 7  # input: major, minor


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
    """A simulated LAN9253 with what src/ethercat_test.cpp does behind it: EEPROM emulation from the
    same SII image, AL state requests acknowledged (BOOT refused). Checks the master's sequences
    without hardware. Like the real one, AL control (0x0120) is a mailbox (ESC configuration 0x0141
    bit 0 = 0, DS00003421 note 8): after a write the master cannot write it again (working counter 0)
    until the firmware has read it - and the firmware polls only every I/O cycle (50 ms)."""

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
        self.holding, self.input = [0] * 120, [0] * 262  # the register map's sizes (holding 20..111, input 0..261)
        self.input[FW_VERSION_MAJOR:FW_VERSION_MAJOR + 2] = [1, 0]
        self.last_sequence = 0

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
        self.serve_registers()
        self.serve_al()

    def serve_registers(self) -> None:
        """serve_registers() of the firmware (ethercat_registers.hpp: execute())."""
        sequence = self.u16(RA_REQUEST)
        if sequence in (0, self.last_sequence):
            return
        self.last_sequence = sequence
        function, start, count = self.reg[RA_REQUEST + 2], self.u16(RA_REQUEST + 4), self.u16(RA_REQUEST + 6)
        table = self.input if function == READ_INPUT else self.holding
        values, exception = [], 0
        if function not in (READ_HOLDING, READ_INPUT, WRITE_HOLDING):
            exception = 1
        elif not 1 <= count <= RA_MAX_COUNT:
            exception = 3
        elif start + count > len(table):
            exception = 2
        elif function == WRITE_HOLDING:
            self.holding[start:start + count] = struct.unpack(f"<{count}H", self.reg[RA_REQUEST + 8:RA_REQUEST + 8 + 2 * count])
        else:
            values = table[start:start + count]
        self.reg[RA_RESPONSE + 2:RA_RESPONSE + 8] = struct.pack("<BBHH", function, exception, start, count)
        self.reg[RA_RESPONSE + 8:RA_RESPONSE + 8 + 2 * len(values)] = struct.pack(f"<{len(values)}H", *values)
        self.reg[RA_RESPONSE:RA_RESPONSE + 2] = struct.pack("<H", sequence)

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


# ---- the SII image of the firmware (same as sii::make() in src/ethercat_sii.hpp) -------------------
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
class Master:
    def __init__(self, link, verbose: bool = False):
        self.link, self.verbose, self.index = link, verbose, 0

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

    def read_sii(self, words: int = 128) -> list[int]:
        self.write(REG["eeprom_config"], b"\x02")  # force: the EEPROM to EtherCAT (SOEM's ec_eeprom2master)
        self.write(REG["eeprom_config"], b"\x00")
        out: list[int] = []
        while len(out) < words:
            data = self.eeprom_read(len(out))
            out += list(struct.unpack(f"<{len(data) // 2}H", data))
        return out[:words]

    @staticmethod
    def print_sii(w: list[int]) -> bool:
        crc_ok = config_crc(w) == (w[7] & 0xFF)
        print(f"SII config area: PDI control {w[0] & 0xFF:#04x}, PDI config {w[1] & 0xFF:#04x}, "
              f"CRC {w[7] & 0xFF:#04x} ({'ok' if crc_ok else 'WRONG, expected %#04x' % config_crc(w)})")
        print(f"SII identity: vendor {w[8] | w[9] << 16:#010x}, product {w[10] | w[11] << 16:#010x}, "
              f"revision {w[12] | w[13] << 16}, size {(w[0x3E] + 1)} Kbit, version {w[0x3F]}")
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
                print(f"SII general: group {body[0]}, image {body[1]}, order {body[2]}, name {body[3]}")
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

    # 5. register access (src/ethercat_registers.hpp): the Modbus register map, in any AL state
    def register_request(self, function: int, start: int, count: int, values: list[int] | None = None,
                         timeout: float = 1.0) -> list[int]:
        values = values or []
        if not 1 <= count <= RA_MAX_COUNT:
            raise MasterError(f"1..{RA_MAX_COUNT} registers per request")
        sequence = self.r16(RA_RESPONSE) % 0xFFFF + 1  # never 0, never the last one
        self.write(RA_REQUEST + 2, struct.pack(f"<BBHH{len(values)}H", function, 0, start, count, *values))
        self.write(RA_REQUEST, struct.pack("<H", sequence))  # last: the trigger
        deadline = time.monotonic() + timeout
        while True:
            answered, _, exception, _, _ = struct.unpack("<HBBHH", self.read(RA_RESPONSE, RA_HEADER))
            if answered == sequence:
                break
            if time.monotonic() > deadline:
                raise MasterError(f"register request: no answer within {timeout} s - does the firmware serve "
                                  "register access (serve_registers(), EtherCAT Direct Mapped)?")
            time.sleep(0.005)
        if exception:
            raise MasterError(f"register request {function:#04x} {start}..{start + count - 1}: Modbus exception "
                              f"{exception} ({MODBUS_EXCEPTIONS.get(exception, '?')})")
        if function in (READ_HOLDING, READ_INPUT):
            return list(struct.unpack(f"<{count}H", self.read(RA_RESPONSE + RA_HEADER, 2 * count)))
        return []

    def read_holding(self, start: int, count: int = 1) -> list[int]:
        return self.register_request(READ_HOLDING, start, count)

    def read_input(self, start: int, count: int = 1) -> list[int]:
        return self.register_request(READ_INPUT, start, count)

    def write_holding(self, start: int, values: list[int]) -> None:
        self.register_request(WRITE_HOLDING, start, len(values), values)


# ---- commands ---------------------------------------------------------------------------------------
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
        if args.command == "regs":
            if args.op == "write":
                m.write_holding(args.start, args.values)
                print(f"holding {args.start}..{args.start + len(args.values) - 1} := {args.values}")
            else:
                read = m.read_input if args.op == "input" else m.read_holding
                for i, v in enumerate(read(args.start, args.count)):
                    print(f"{args.op} {args.start + i}: {v} ({v:#06x})")
            return 0
        if args.command == "blink":
            # strip (CH1) pattern 1 = red, single LED (CH2) palette 1 = red; 0 = off
            on = False
            try:
                n = 0
                while args.cycles == 0 or n < 2 * args.cycles:
                    on = not on
                    m.write_holding(WS2812_CH1_PATTERN, [args.pattern if on else 0, args.colour if on else 0])
                    print(time.strftime("%H:%M:%S"), "LEDs", "on" if on else "off", flush=True)
                    n += 1
                    time.sleep(args.interval)
            except KeyboardInterrupt:
                pass
            m.write_holding(WS2812_CH1_PATTERN, [0, 0])
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
        for state in ("init", "preop", "safeop", "op", "init"):
            ok = m.request_state(state) and ok
        refused = not m.request_state("boot")
        print(f"  BOOT {'refused as expected' if refused else 'accepted - the test firmware should refuse it'}")
        ok = ok and refused
        major, minor = m.read_input(FW_VERSION_MAJOR, 2)
        print(f"register access: firmware version {major}.{minor} (input registers {FW_VERSION_MAJOR}..{FW_VERSION_MAJOR + 1})")
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
    rg = sub.add_parser("regs", help="read/write the Modbus register map over EtherCAT (docs/generated/modbus-registers.md)")
    rg_ops = rg.add_subparsers(dest="op", required=True)
    for name in ("holding", "input"):
        r = rg_ops.add_parser(name, help=f"read {name} registers")
        r.add_argument("start", type=int)
        r.add_argument("count", type=int, nargs="?", default=1)
    w = rg_ops.add_parser("write", help="write holding registers")
    w.add_argument("start", type=int)
    w.add_argument("values", type=lambda s: int(s, 0), nargs="+")
    bl = sub.add_parser("blink", help="RGB LEDs on/off over EtherCAT (holding 100/101; Ctrl+C ends, LEDs off)")
    bl.add_argument("--interval", type=float, default=1.0, help="seconds per on/off phase (default 1)")
    bl.add_argument("--cycles", type=int, default=0, help="on/off cycles, 0 = until Ctrl+C")
    bl.add_argument("--pattern", type=int, default=1, help="strip pattern when on (1 red, 2 yellow, 3 green, 4 blue/white)")
    bl.add_argument("--colour", type=int, default=1, help="single LED palette index when on (1 red, 2 green, 3 blue, ...)")
    sub.add_parser("test", help="end to end: scan, SII, INIT -> PRE-OP -> SAFE-OP -> OP -> INIT, BOOT refused")
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
