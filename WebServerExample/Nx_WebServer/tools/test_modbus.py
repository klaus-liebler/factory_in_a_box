#!/usr/bin/env python3
"""Testwerkzeug fuer den Modbus-TCP-Server der Control-Unit (Nx_WebServer).

Voraussetzung: pip install pymodbus

Beispiele:
    python test_modbus.py --host 192.168.1.50
    python test_modbus.py --host 192.168.1.50 --set-valve 1 on
    python test_modbus.py --host 192.168.1.50 --set-valve 1 off
"""
import argparse
import sys
import time

from pymodbus.client import ModbusTcpClient

MODBUS_PORT = 502

# Register-Adressen, siehe Core/Src/modbus_register_map.hpp
INPUT_HEALTH_STATE = 0
INPUT_CHIP_ID_W0_HI = 1
INPUT_FW_VERSION_MAJOR = 7
INPUT_TIMER_TICK = 10
INPUT_ETH_LINK_STATUS = 80
INPUT_LIGHTBARRIER1 = 180
INPUT_PRESSURE_RAW = 200

HOLDING_VALVE1 = 20

HEALTH_BITS = {
    0: "OK",
    1: "OVERTEMPERATURE",
    2: "POWER_FAULT",
    3: "CAN_ERROR",
    4: "ETH_LINK_DOWN",
    5: "I2C_SENSOR_FAULT",
    6: "STEPPER_FAULT",
}

ETH_SPEEDS = {0: "10M", 1: "100M", 2: "1000M"}


def check(result, what):
    if result.isError():
        print(f"FEHLER bei {what}: {result}")
        sys.exit(1)
    return result


def dump_status(client: ModbusTcpClient) -> None:
    health = check(client.read_input_registers(INPUT_HEALTH_STATE, count=1), "HEALTH_STATE").registers[0]
    flags = [name for bit, name in HEALTH_BITS.items() if health & (1 << bit)] or ["(keine Flags)"]
    print(f"HEALTH_STATE   : 0x{health:04x}  [{', '.join(flags)}]")

    chip_id_words = check(client.read_input_registers(INPUT_CHIP_ID_W0_HI, count=6), "CHIP_ID").registers
    uid_hex = "".join(f"{w:04x}" for w in chip_id_words)
    print(f"CHIP_ID        : {uid_hex}")

    fw = check(client.read_input_registers(INPUT_FW_VERSION_MAJOR, count=3), "FW_VERSION").registers
    print(f"FW_VERSION     : {fw[0]}.{fw[1]}.{fw[2]}")

    tick0 = check(client.read_input_registers(INPUT_TIMER_TICK, count=1), "TIMER_TICK").registers[0]
    time.sleep(0.2)
    tick1 = check(client.read_input_registers(INPUT_TIMER_TICK, count=1), "TIMER_TICK").registers[0]
    moving = "laeuft" if tick1 != tick0 else "STEHT (unerwartet!)"
    print(f"TIMER_TICK     : {tick0} -> {tick1}  ({moving})")

    eth = check(client.read_input_registers(INPUT_ETH_LINK_STATUS, count=3), "ETH_LINK").registers
    link_up = "up" if eth[0] else "down"
    speed = ETH_SPEEDS.get(eth[1], f"? ({eth[1]})")
    duplex = "full" if eth[2] else "half"
    print(f"ETH_LINK       : {link_up}, {speed}, {duplex}")

    ls = check(client.read_input_registers(INPUT_LIGHTBARRIER1, count=2), "LIGHTBARRIER").registers
    print(f"LIGHTBARRIER1/2: {ls[0]} / {ls[1]}")

    pressure = check(client.read_input_registers(INPUT_PRESSURE_RAW, count=1), "PRESSURE_RAW").registers[0]
    print(f"PRESSURE_RAW   : {pressure}")

    valves = check(client.read_holding_registers(HOLDING_VALVE1, count=3), "VALVE1..3").registers
    print(f"VALVE1/2/3     : {valves[0]} / {valves[1]} / {valves[2]}")


def set_valve(client: ModbusTcpClient, valve_number: int, state: bool) -> None:
    address = HOLDING_VALVE1 + (valve_number - 1)
    check(client.write_register(address, 1 if state else 0), f"VALVE{valve_number} schreiben")
    readback = check(client.read_holding_registers(address, count=1), f"VALVE{valve_number} lesen").registers[0]
    print(f"VALVE{valve_number} gesetzt auf {'AN' if state else 'AUS'} (Register liest jetzt: {readback})")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", required=True, help="IP-Adresse der Control-Unit")
    parser.add_argument("--port", type=int, default=MODBUS_PORT)
    parser.add_argument(
        "--set-valve",
        nargs=2,
        metavar=("N", "on|off"),
        help="Ventil N (1-3) real schalten, z.B. --set-valve 1 on. Bewegt echte Hardware!",
    )
    args = parser.parse_args()

    client = ModbusTcpClient(args.host, port=args.port)
    if not client.connect():
        print(f"Verbindung zu {args.host}:{args.port} fehlgeschlagen.")
        sys.exit(1)

    try:
        if args.set_valve:
            valve_number = int(args.set_valve[0])
            state_str = args.set_valve[1].lower()
            if valve_number not in (1, 2, 3) or state_str not in ("on", "off"):
                parser.error("--set-valve erwartet: N in {1,2,3} und on/off")
            set_valve(client, valve_number, state_str == "on")
        else:
            dump_status(client)
    finally:
        client.close()


if __name__ == "__main__":
    main()
