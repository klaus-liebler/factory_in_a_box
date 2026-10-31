# Firmware architecture (rtoos)

The Factory Control Unit firmware runs on [rtoos](https://github.com/klaus-liebler/rtoos) (submodule `rtoos/`): GCC 16, C++26, no HAL, no heap, every peripheral configured at compile time. The builder (`builder/`, [build-process.md](build-process.md)) provisions the board and generates the register map, the WebSocket protocol and the identity headers.

## Threads

| Thread | Priority | Does |
|---|---|---|
| `io` | 10 | boot: reset cause, identity check, banner, supply gate (INA226, USB-PD), power consumers; then the I/O cycle every 50 ms |
| `net` | 8 | IP stack and every network service (one `NetRuntime`) |
| `log` | 14 | writes the log journal to the console (USART3) |

Stepper step timing runs in TIM17/TIM16 interrupts (priority 4, never masked); USB-PD in UCPD1/TIM5 (priority 7).

## Boot order (as `App::AppThread` before)

1. Chip UID must match `Core/generated/device_ids.hh` (else the LED blinks fast and nothing starts).
2. Supply gate: if the INA226 measures less than 18 V, the board asks a USB-C supply for 20 V (USB Power Delivery sink, rtoos `usb_pd`) and waits.
3. Power consumers: pins (`src/pins.hpp`), I2C bus scans, sensors, steppers (TMC2209), servos, LEDs, CAN, ADC.
4. Network thread, then the I/O cycle.

## Sources

| File | Contents |
|---|---|
| `src/pins.hpp` | pins per hardware revision (1: first boards, 2: current PCB); named by signal, checked against ST's pin database at compile time |
| `src/hardware.hpp` | all peripheral and chip types with their compile-time configuration |
| `src/boot.*` | reset cause, identity, banner, supply gate, USB-PD |
| `src/io.*` | the I/O cycle: inputs, sensors (INA226, VL53L0X x3, TCS34725, HX711, DTS), outputs (valves, vacuum, PWM), steppers with sensorless homing, WS2812, CAN, health bits, heartbeat |
| `src/roarm/*` | RoArm-M3: kinematics, joint tracking, teach mode, missions in the internal flash (littlefs, same files as before) |
| `src/network.*` | Ethernet (DHCP, mDNS, SNTP), USB CDC-NCM + DHCP server and CDC-ACM Modbus RTU, HTTPS/WebSocket, Modbus TCP, OPC UA, SSH log terminal |
| `src/web.*` | WebSocket protocol (`ws_protocol.hh`): register panel, system info; RoArm requests go to the I/O thread |
| `src/task_monitor.*` | task manager: threads (state, stack, CPU share) and pools |

## Interfaces

- Modbus TCP 502, Modbus RTU over USB CDC-ACM (unit 1), OPC UA 4840 (`Objects/FactoryControlUnit`): the register map from `register_map_schema/*.cs`.
- `https://<hostname>.local/` web app, `wss://<hostname>.local/ws`.
- `ssh log@<hostname>.local`: the log terminal (boot log, then live; `q` ends it). Keys: builder setting `Ssh.AuthorizedKeyFiles`.
- USB: the PC gets 192.168.173.2, the board is 192.168.173.1.

## Behaviour changes from the HAL firmware

- A new stepper target starts when the current move has finished (no change of plan mid-ramp).
- RoArm mission requests of the web UI are executed by the I/O thread (the flash belongs to it) and answered through the WebSocket outbox.
- The log goes to the SSH terminal, no longer to the WebSocket.
- The USB NCM interface of the board uses the PC's MAC address with the lowest bit flipped (the two ends differ).
- CAN runs at 500 kbit/s (the CubeMX values gave no meaningful bit rate).
