# Firmware architecture (rtoos)

The Factory Control Unit firmware runs on [rtoos](https://github.com/klaus-liebler/rtoos) (submodule `rtoos/`): GCC 16, C++26, no HAL, no heap, every peripheral configured at compile time. The builder (`builder/`, [build-process.md](build-process.md)) provisions the board and generates the register map, the WebSocket protocol and the identity headers.

## Threads

| Thread | Priority | Does |
|---|---|---|
| `io` | 10 | boot: reset cause, identity check, banner, supply gate (INA226, USB-PD), power consumers; then the I/O cycle every 50 ms |
| `net` | 8 | IP stack and every network service (one `NetRuntime`) |
| `log` | 14 | writes the log journal to the console (USART3) |

Stepper step timing runs in TIM17/TIM16 interrupts (priority 4, never masked); USB-PD on revision 1 in UCPD1/TIM5 (priority 7). Conveyor and compressor PWM: TIM4, 1 kHz.

## Boot order (as `App::AppThread` before)

1. Chip UID must match `Core/generated/device_ids.hh` (else the LED blinks fast and nothing starts).
2. Supply gate: if the INA226 measures less than 18 V, the board waits for 20 V from a USB-C supply. Revision 2: a CH224K negotiates 20 V by itself (CFG1 open), the firmware only waits; revision 1: USB Power Delivery sink on UCPD1 (rtoos `usb_pd`).
3. Power consumers: pins (`src/pins.hpp`), I2C bus scans, sensors, steppers (TMC2209), servos, LEDs, CAN, ADC.
4. Network thread, then the I/O cycle.

## Sources

| File | Contents |
|---|---|
| `src/pins.hpp` | pins per hardware revision (1: first boards, 2: current PCB with CH224K); named by signal, checked against ST's pin database at compile time |
| `src/hardware.hpp` | all peripheral and chip types with their compile-time configuration |
| `src/boot.*` | reset cause, identity, banner, supply gate, USB-PD |
| `src/io.*` | the I/O cycle: inputs, sensors (INA226, VL53L0X x3, TCS34725, HX711, DTS), outputs (valves, vacuum, PWM), steppers with sensorless homing, WS2812, CAN, health bits, heartbeat |
| `src/roarm/*` | RoArm-M3: kinematics, joint tracking, teach mode, missions in the internal flash (littlefs, same files as before) |
| `src/network.*` | Ethernet (DHCP, mDNS, SNTP), USB CDC-NCM + DHCP server and CDC-ACM Modbus RTU, HTTPS/WebSocket, Modbus TCP, OPC UA |
| `src/web.*` | WebSocket protocol (`ws_protocol.hh`): register panel, system info, log records to every browser; RoArm and terminal requests go to the I/O thread |
| `src/terminal.*` | the commands of the web UI's Terminal page (inputs, outputs, PWM, GPIO, ADC, supply, sensors, I2C scan, registers, steppers, LEDs, reset) and the live value lookups; the catalog is `terminal_schema/commands.json` |
| `src/task_monitor.*` | task manager: threads (state, stack, CPU share) and pools |

## Interfaces

- Modbus TCP 502, Modbus RTU over USB CDC-ACM (unit 1), OPC UA 4840 (`Objects/FactoryControlUnit`): the register map from `register_map_schema/*.cs`.
- `https://<hostname>.local/` web app, `wss://<hostname>.local/ws`.
- Web UI page "Terminal": a console for the commands above and the live log as a table (level filter, auto-scroll, 10000 lines kept in the browser). The command catalog `terminal_schema/commands.json` is compiled into the web app (builder step `GenerateTerminalFiles`): history (arrow up), completion of commands and parameter values (Tab), the parameter tooltip, `help` and `clear` run in the browser. A parameter's values are listed in the catalog (`values`), computed in the browser (`source`: command names, registers) or fetched live from the device (`lookup`: `terminal.LookupRequest`, answered by `src/terminal.cpp`).
- Adding a command: an entry in `commands.json`, a handler function and an entry in `handlers` (`src/terminal.cpp`); a missing handler or lookup answer is a compile error.
- USB: the PC gets 192.168.173.2, the board is 192.168.173.1.

## Behaviour changes from the HAL firmware

- A new stepper target starts when the current move has finished (no change of plan mid-ramp).
- RoArm mission requests of the web UI are executed by the I/O thread (the flash belongs to it) and answered through the WebSocket outbox.
- The log goes to the web UI's Terminal page as a table (before: only into the browser's developer console).
- The USB NCM interface of the board uses the PC's MAC address with the lowest bit flipped (the two ends differ).
- CAN runs at 500 kbit/s (the CubeMX values gave no meaningful bit rate).
