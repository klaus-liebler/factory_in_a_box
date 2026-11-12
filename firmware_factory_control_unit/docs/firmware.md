# Firmware architecture (rtoos)

The Factory Control Unit firmware runs on [rtoos](https://github.com/klaus-liebler/rtoos) (submodule `rtoos/`): GCC 16, C++26, no HAL, no heap, every peripheral configured at compile time. Hardware revisions 1/2 have an STM32H563 (Cortex-M33, 250 MHz), revision 3 an STM32H743VIT6 (Cortex-M7, 400 MHz); the board package's `board/chip.hpp` names the chip, `src/pins.hpp` and `src/hardware.hpp` have a branch per chip, everything else is the same code. USB (CDC-NCM network + CDC-ACM Modbus RTU) runs on both: the H563's USB_DRD_FS, the H743's USB2_OTG_FS (`board::UsbDevice`); on revision 3 tested in Renode only (`tools/test-usb-renode.py`), not yet on the board. The builder (`builder/`, [build-process.md](build-process.md)) provisions the board and generates the register map, the WebSocket protocol and the identity headers.

## Threads

| Thread | Priority | Does |
|---|---|---|
| `io` | 10 | boot: reset cause, identity check, banner, supply gate (INA226, USB-PD), power consumers; then the I/O cycle every 50 ms |
| `net` | 8 | IP stack and every network service (one `NetRuntime`) |
| `plc` | 11 | the IEC 61131-3 program ([plc-iec61131.md](plc-iec61131.md)): one cycle after each I/O cycle; program and control source in the flash slots at `0x081C0000` |
| `log` | 14 | writes the log journal to the console (USART3) |

Stepper steps come from TIM2/TIM3 compare outputs, their periods from DMA (STM32H563: GPDMA1 channels 4/5, 16-bit items widened; STM32H743: DMA1 streams 2/3, 32-bit items) (rtos `TimerStepper`; the DMA interrupt refills every 32 steps, priority 4, never masked); TMC2209 DEDGE: one microstep per STEP edge; I2C buses interrupt-driven (revisions 1/2: I2C1/2/4, revision 3: I2C1/3/4) (the I/O thread sleeps on a semaphore while the bytes move, priority 6); HX711 read by interrupts alone (EXTI on DOUT starts the SPI transfer, its end-of-transfer interrupt decodes); INA226 ALERT (conversion ready, every 35.2 ms) wakes the I/O thread between cycles; light barriers latched by EXTI (an interruption shorter than a cycle still shows for one cycle); USB-PD on revision 1 in UCPD1/TIM5 (priority 7). Conveyor and compressor PWM: TIM4, 1 kHz.

## Boot order (as `App::AppThread` before)

1. Chip UID must match `generated/device_ids.hh` (else the LED blinks fast and nothing starts).
2. Supply gate: if the INA226 measures less than 18 V, the board waits for 20 V from a USB-C supply. Revision 2: a CH224K negotiates 20 V by itself (CFG1 open), the firmware only waits; revision 3 likewise with a CH224Q (CFG1; its I2C on I2C3 is not used); revision 1: USB Power Delivery sink on UCPD1 (rtoos `usb_pd`).
3. Power consumers: pins (`src/pins.hpp`), I2C bus scans, sensors, steppers (TMC2209), servos, LEDs, CAN, ADC.
4. Network thread, then the I/O cycle.

## Sources

| File | Contents |
|---|---|
| `src/pins.hpp` | pins per hardware revision (1: first boards, 2: STM32H563 with CH224K, 3: STM32H743); named by signal, checked against ST's pin database at compile time |
| `src/hardware.hpp` | all peripheral and chip types with their compile-time configuration |
| `src/boot.*` | reset cause, identity, banner, supply gate, USB-PD |
| `src/io.*` | the I/O cycle: inputs, sensors (INA226, VL53L0X x3, TCS34725, HX711, CPU temperature: DTS on the H563, ADC3's sensor on the H743), outputs (valves, vacuum, PWM), steppers with sensorless homing, WS2812, CAN, health bits, heartbeat |
| `src/roarm/*` | RoArm-M3: kinematics, joint tracking, teach mode, missions on the SD card (FAT, `/MISSIONS/<index>.MSN`; STM32H563 boards move their former flash missions onto the card at the first start); end-to-end test in Renode without hardware: `tools/test-missions-sd-renode.py` (revision 3 build, TAP adapter) |
| `src/malibotti.*` | the Malibotti robot on USART6 (revision 3, J16): missions started over `MalibottiMissionControl`, its state in `MalibottiStatus`/`MalibottiActiveMission`; COBS+CRC framed BestBinaryBuffers messages - [malibotti-uart.md](malibotti-uart.md) |
| `src/ethercat_test.*` | LAN9253 bring-up test (revision 1 and 3): SPI by GPIO, ESC probe at the start, EEPROM emulation with a minimal SII image (vendor 0, product 0x9253, "FCU EtherCAT test"), AL states INIT/PRE-OP/SAFE-OP/OP acknowledged without checks, link changes per port - all in the log. A master (TwinCAT, SOEM) should find and identify the device; no mailbox, no process data |
| `src/network.*` | Ethernet (DHCP, mDNS, SNTP), USB CDC-NCM + DHCP server and CDC-ACM Modbus RTU, HTTPS/WebSocket, Modbus TCP, OPC UA |
| `src/web.*` | WebSocket protocol (`ws_protocol.hh`): register panel, system info, log records to every browser; RoArm and terminal requests go to the I/O thread |
| `src/terminal.*` | the commands of the web UI's Terminal page (inputs, outputs, PWM, GPIO, ADC, supply, sensors, I2C scan, registers, steppers and their homing, LEDs, reset) and the live value lookups; the catalog is `schema/terminal/commands.cs` |
| `src/task_monitor.*` | task manager: threads (state, stack, CPU share) and pools |

## Interfaces

- Modbus TCP 502, Modbus RTU over USB CDC-ACM (unit 1), OPC UA 4840 (`Objects/FactoryControlUnit`): the register map from `schema/register_map/*.cs`.
- PLC (IEC 61131-3, `src/plc.*`): programmed and debugged in the web UI's page "SPS (IEC 61131)", runs on the device without a browser. Control source "IEC 61131": the program's outputs are refused to Modbus/OPC UA/web UI/terminal (Modbus exception 0x02, OPC UA Bad_NotWritable), STOP/ERROR put valves, pumps, PWM and LEDs to 0. End-to-end test in Renode without hardware: `python tools/test-plc-renode.py [--revision 1]` (all commands: [kommandos.md](kommandos.md)).
- `https://<hostname>.local/` web app, `wss://<hostname>.local/ws`.
- Web UI page "Terminal": a console for the commands above and the live log as a table (level filter, auto-scroll, 10000 lines kept in the browser). How the console works, its limits and how to add a command: [terminal.md](terminal.md).
- Steppers: at the start both home at once (sensorless, StallGuard, towards the negative end stop) as soon as the motor supply is up and the TMC2209 answers; the end stop becomes position 0. Result in `StepperXStartHoming` (0 ok, 2 no end stop within 10 s, 3 driver does not answer) and in the status (Bit4 homing, Bit5 homed, Bit2 failed); write 1 (or `home <motor>` in the terminal) to home again. Without the motor supply there is no homing at the start. An axis that is not homed does not move: its target waits; a successful homing sets the target to 0. StallGuard is polled every 5 ms during the homing over UART; with the DIAG outputs wired (revision 2: PD6/PE0, revision 3: PD6/PE4, `pins::stepper1_diag`/`stepper2_diag`) a rising DIAG wakes the I/O thread by interrupt and the motor stops about 1 ms after the stall. DIAG also rises on a driver error: such a stop reads DRV_STATUS and counts as a failed homing. A stall that only the polling saw while DIAG is configured is logged as a warning (missing DIAG wire?).
- Web UI page "Schrittmotoren" (`ws::stepper`, `schema/best_binary_buffers/stepper.cs`; the I/O thread answers): the test bench of both axes. Manual control (the register targets rest, moves also without homing; switching it off makes the position the target), moves of N steps with a speed profile chosen at run time - start and cruise rate, acceleration, linear ramp or sigmoid (rtoos `ProfileStepper`: the periods are computed during the move, the same profile then also drives the register targets) -, stop, sensorless homing with its own speed, SGTHRS and timeout, and every TMC2209 setting (IRUN, IHOLD, IHOLDDELAY, TPOWERDOWN, StealthChop/SpreadCycle, TPWMTHRS, pwm_autoscale/autograd, micro steps, SGTHRS, TCOOLTHRS, CoolStep). Live: position, CS_ACTUAL, SG_RESULT, PWM_SCALE, TSTEP and the DRV_STATUS flags, as curves. The settings live in RAM (defaults at every start). Revision 1 has no DIAG wires: its homing sees the stall by polling SG_RESULT over UART every 5 ms (the page says which way stopped the last homing).
- GPIO 1/2 (revision 3, J30/J31, PE2/PE3): binary inputs, active high with the internal pull-down, in the input registers `Gpio1`/`Gpio2` (185/186) - so also PLC inputs and FBD sensors.
- Web UI shell: a common status bar at the bottom (WebSocket state, the current app's mode, the app's own fields; apps publish with `setAppStatus`, `web/src/shell/status-bar.ts`), modal dialogs for every app instead of the browser's confirm/prompt (`web/src/shell/dialogs.ts`), a dark mode switch (kept in the browser), and maximizing also enters the browser's full screen (Esc leaves it).
- At most two HTTPS/WebSocket sessions at once (`net::max_https_sessions`); a third browser is refused.
- OPC UA accepts only clients whose certificate was issued by the CA the firmware trusts (`build/assets/root_ca.der`); `tools/test_opcua.py --ca-cert/--ca-key` issues one for the test client.
- USB: the PC gets 192.168.173.2, the board is 192.168.173.1.

## Behaviour changes from the HAL firmware

- A new stepper target starts when the current move has finished (no change of plan mid-ramp).
- RoArm mission requests of the web UI are executed by the I/O thread (the flash belongs to it) and answered through the WebSocket outbox.
- The log goes to the web UI's Terminal page as a table (before: only into the browser's developer console).
- The USB NCM interface of the board uses the PC's MAC address with the lowest bit flipped (the two ends differ).
- CAN runs at 500 kbit/s (the CubeMX values gave no meaningful bit rate).
