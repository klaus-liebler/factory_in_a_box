# IEC 61131-3 programming and runtime environment (specification)

Status: **draft, review round 1 incorporated** (2026-11-08). No code yet. This file is the reference:
decisions are made here first and the code follows. Questions decided in the review and points
deliberately postponed are under [§14](#14-entscheidungen-aus-dem-review-und-zurückgestellte-punkte).

## 1. Goal

The FCU gets a PLC that is programmed **in the browser**:

- Languages: **ST** (Structured Text), **FBD** (Function Block Diagram, "FUP"), **SFC** (Sequential
  Function Chart, "AS"), following IEC 61131-3 Ed. 3.0 (subset, see [§5](#5-sprachumfang)).
- **Compiling in the browser** (TypeScript): source → machine-near code (bytecode, see
  [§3](#3-grundsatzentscheidung-bytecode-vm-statt-nativem-maschinencode)) + debug information. The
  device only receives the finished image; it neither compiles nor parses.
- **Debugging in the browser**: variable view (live), writing/forcing, breakpoints,
  single step, active SFC steps and FBD signal values live.
- **Control source switch**: the outputs are switched either by the IEC program
  **or** over Modbus/OPC UA (as today). Never both at once.
- **The program runs autonomously on the STM32.** The browser is only needed for programming and
  debugging. At run time no browser has to be open and no PC has to be connected: the image and
  the configuration are in the device's flash, the program starts by itself after power-on
  (autostart, [§4.3](#43-boot)) and keeps running when every browser disconnects. A broken
  WebSocket connection never changes the PLC's state (exception: a debug lease is released,
  but a halted program stays halted, [§10.3](#103-online-debugging)).

Out of scope (deliberately): LD (ladder) and IL (IL is deprecated in Ed. 3); OOP extensions
(METHOD, INTERFACE, INHERITANCE); several resources; online change in the first stage;
safety PLC (no SIL claim).

"FB" in the task is read as **FBD** (the graphical language). Function blocks
(FUNCTION_BLOCK as a POU type) are naturally in scope too.

## 2. Overall architecture

```
 Browser (TypeScript)                                       FCU (rtoos, C++26)
 ┌──────────────────────────────────────────────┐          ┌────────────────────────────────────┐
 │ IDE (lit)                                     │          │ net thread (prio 8)                │
 │  ST editor (CodeMirror 6)                     │  HTTPS   │  /api/plc/*   image, project, cfg  │
 │  FBD editor (SVG)    SFC editor (SVG)         │ ───────▶ │  WS /ws       debug messages       │
 │  Project tree, I/O view, watch, debug control │  WSS     │  Modbus / OPC UA (write filter)    │
 │                                               │ ◀──────▶ │                                    │
 │ Compiler (web/src/plc/compiler, DOM-free)     │          │ PLC threads (prio 11..13, pool)    │
 │  ST parser ─┐                                 │          │  PLC VM: image, data, debug        │
 │  FBD → AST ─┼─▶ semantics ─▶ IR ─▶ bytecode   │          │  process image in/out              │
 │  SFC → AST ─┘                 + debug info    │          │                                    │
 │                                               │          │ io thread (prio 10, 50 ms)         │
 │ Simulator: VM in TypeScript (same ISA)        │          │  inputs → registers → outputs      │
 └──────────────────────────────────────────────┘          └────────────────────────────────────┘
```

Principles:

1. **One instruction set, two VMs.** The bytecode ISA is defined once (opcode table as a
   single source of truth, from which the C++ and TS sides are generated, like
   BestBinaryBuffers/UniversalRegisterAccess). The C++ VM runs on the target and in host tests,
   the TS VM in the browser simulator and in the compiler tests. Both are tested against each other
   (differential tests, [§12](#12-teststrategie)).
2. **The register map stays the hub.** The PLC reads and writes the same
   `registers` bank as Modbus/OPC UA. `io.cpp` stays the only part that touches the hardware;
   the PLC does not know pins.
3. **IEC 61131 is an integral part of rtoos and the web UI, not a separate product.**
   - rtoos (`rtoos/include/rtos/plc/`, `rtoos/src/plc/`): VM, image format, task scheduler, debug
     core, the opcode table (single source of truth). Generic, host-testable, usable by other rtoos
     devices.
   - Firmware (`src/plc.*`): binding to the register map, control source, natives of the FCU library.
   - Web UI (`web/src/plc/`): compiler (in its own DOM-free subfolder `web/src/plc/compiler/`,
     which runs in vitest and Node too), TS VM/simulator, editors, debugger. Part of the existing
     web build, no separate npm package.

## 3. Key decision: bytecode VM instead of native machine code

**Decision E1:** The compiler produces **bytecode for a VM** in the firmware, not Thumb-2 machine code.
Native code generation stays possible as a later option (same IR, its own back end,
[§13](#13-phasen) P7) if the measured performance is not enough.

| Criterion | Bytecode VM | Native Thumb-2 |
|---|---|---|
| Breakpoints / single step | VM state lives in a struct, halting mid-cycle is trivial; breakpoint = flag per instruction | needs a debug monitor, BKPT patching in RAM code, context save, MPU |
| Isolation | every access checked against segment bounds; an error stops the PLC, not the firmware | a wrong pointer/jump hits the firmware (HardFault); needs MPU regions + an unprivileged thread |
| Endless loop / overrun | instruction budget per cycle, deterministic (emulator-friendly) | time watchdog only |
| Portability | one ISA for H563 (M33), H743 (M7), host, emulators, browser | two back ends (FPU: M33 single only, M7 double), position-independent code, relocation |
| Simulation in the browser | the same VM in TS | needs an extra interpreter or ARM emulator in the browser |
| Speed | ~20–40 cycles per instruction (switch dispatch) | 3–10× faster |

Estimate: at 250 MHz (H563) ≥ 5 M bytecode instructions/s. A typical training/factory program
(< 5 000 instructions per cycle) needs < 1 ms per cycle, a fraction of the 50 ms I/O cycle.
"Machine-near" is met: the image contains no parsing work, only resolved addresses,
typed operations and fixed jump targets.

## 4. Runtime model on the device

### 4.1 IEC tasks and RTOS threads

IEC 61131-3 knows several tasks per resource, each with an interval (cyclic) or a trigger
(SINGLE, event) and a priority; program instances are assigned to the tasks. The standard allows
non-preemptive and preemptive scheduling.

**Decision E2: no strict 1:1 mapping of IEC task → RTOS thread, but a fixed thread pool.**

- Why not 1:1: rtoos has no heap at run time. Threads, their stacks and `max_threads`
  are fixed when the firmware is compiled, but the IEC tasks only come with the downloaded image.
  1:1 would need either "as many threads as the maximum number of tasks" (every one with its own stack, mostly unused)
  or dynamic threads.
- Instead: the VM is **reentrant**. Code and data segments are shared; each IEC task has its
  own **execution context** (PC, operand stack, frame chain, its own stack area in the data
  segment). A context can run in any thread.
- The firmware has a pool of `plc_threads` RTOS threads (compile-time parameter, Stage 1: 1,
  target: 3) with fixed priorities **11, 12, 13**: below `io` (10), so the I/O cycle and the
  steppers never wait for the PLC, and above `log` (14). The compiler distributes the IEC tasks by their
  priority over the pool: tasks of the same priority class share a thread and run there
  **non-preemptively** in a fixed order; different classes **preempt** each other
  through the RTOS. With up to 3 different priorities this is effectively 1:1; with more the
  compiler merges neighboring classes and reports it.
- Every PLC thread sleeps until the next due time of its tasks (kernel timer, 1 ms
  resolution). A task not finished at its next due time counts as an overrun ([§4.2](#42-betriebszustände)).
- Data consistency between tasks as in the standard: no automatic locks. Single accesses up to
  32 bits are atomic; LINT/LREAL/structures are not. The compiler **warns** when a global variable
  is written in one task and read in another, and **errors out** when two tasks write the same output.
- Process image per task: a task copies at its start only the inputs it uses and writes at its end
  only its outputs (the compiler knows both per task, the image carries them in the task table).
- `KernelConfig::max_threads` grows by `plc_threads` (Stage 1: 5 → 6).

**Stage 1 (P2):** exactly one cyclic task, one PLC thread, coupled to the I/O cycle (below).
The VM context, image format (task table) and compiler are built for several tasks from
P1 on, so P6 needs no redesign.

**Several tasks with their own cycle times (P6)** additionally need fresh inputs: the
register bank is only updated by the I/O cycle every 50 ms; a 10 ms task would see the same
sensor value five times. Plan for that:

- **Fast I/O** directly in the task's process image, bypassing the register bank: digital inputs
  (light barriers incl. EXTI latch), valves, PWM duty, pressure ADC. `io` provides
  thread-safe functions for them (each is a single atomic register access: GPIO IDR, BSRR, CCR, ADC DR);
  `io` stays the owner of the hardware configuration.
- **Slow I/O** (ToF, color, scale, INA226, CAN status) stays on the I/O cycle. The register schema gets
  the update period per register (new attribute `UpdateMs`); the IDE shows it for each I/O
  variable and warns when a task faster than that reads it.

**Stage 1 cycle** (one task, coupled to the I/O cycle):

```
io thread (every 50 ms)                          plc thread
  inputs(), sensors, ...                          wait for notify
  ── takes the PLC's output image ──┐
  outputs()                         │
  notify(plc) ─────────────────────────────────▶  1. input image  ← registers (snapshot under Guard)
                                    │             2. run program (all programs of the task)
                                    └──────────── 3. output image → registers (only PLC-owned outputs)
```

- Process image per IEC: inputs are frozen at the start of the cycle, outputs only become visible at the end
  of the cycle. Latency input → output: ≤ 1 I/O cycle (50 ms) + PLC run time.
- Task interval = n × I/O cycle (n ≥ 1, default 1 → 50 ms). 50 ms as the fastest cycle is enough for
  Stage 1 (review F1); faster tasks come with several tasks in P6 (above).
- Not finished within the interval (overrun): counted, after `max_overruns` (default 3 in a row)
  → ERROR. Per cycle there is also an instruction budget (default 2 M); exceeded → ERROR
  (catches endless loops deterministically, even in the emulators).
- Time base: `kernel.now()` (ms) for TIME/timers, read at the start of the cycle (all TONs in a cycle
  see the same time). DATE/TOD/DT from the SNTP clock; without SNTP sync these values are marked
  invalid (status bit) and run from boot time 1970-01-01.

### 4.2 Operating states

```
          download / reset             RUN
 NO_PROGRAM ──────────▶ STOP ◀──────────────▶ RUN
                          ▲      STOP          │ breakpoint / step
                          │                    ▼
                          │                 HALTED (debug, mid-cycle)
                          │  reset             │ runtime error / overrun / budget
                          └──────── ERROR ◀────┘
```

| State | Program | PLC-owned outputs |
|---|---|---|
| NO_PROGRAM | none loaded | safe state |
| STOP | loaded, not running | safe state |
| RUN | cyclic | from the program |
| HALTED | held at a breakpoint/step | **frozen** (last committed output image), I/O cycle keeps running; option "safe state on halt" per project |
| ERROR | stopped, error location (POU, line) kept | safe state |

Reset kinds: **warm** (variables to initial values, RETAIN kept), **cold** (RETAIN
initialized too). Variables keep their values on STOP→RUN.

**Safe state**: for each output, the safe value from the register schema (new attribute
`SafeValue`, default 0: valves closed, PWM 0, steppers: no new target, see [§6.3](#63-registerklassen-ausgang-kommando-parameter)).

### 4.3 Boot

1. Boot as today (identity, supply gate, power consumers, homing starts in `io`).
2. `plc` mounts the program store, checks the image (magic, CRC, ISA version, I/O map hash).
3. Control source = the persisted mode. In IEC mode with *autostart* and a valid image: RUN
   at the first I/O cycle, **without any browser or network connection** (the network may even come
   up later or not at all). Without a valid image: IEC mode stays, state NO_PROGRAM/ERROR,
   outputs safe (no silent fallback to fieldbus).
4. Hardware acceptance test for this: power-cycle the device without a network cable/USB, the program
   runs (visible at the outputs and LEDs) — part of P2 ([§13](#13-phasen)).

### 4.4 Resources (budget, configurable at compile time)

| | H563 (rev 1/2) | H743 (rev 3) | Today used/free |
|---|---|---|---|
| Code segment (RAM, the image is copied there) | 64 KB | 64 KB | H563 RAM: 265 KB of 576 KB |
| Data segment (globals, instances, stack) | 32 KB | 32 KB | H743 AXI RAM: 261 KB of 512 KB |
| VM in flash | ≤ 40 KB | ≤ 40 KB | flash: ~500 KB of 2 MB |
| Stack per PLC thread | ≤ 4 KB (size from the frames, rtoos rule; the IEC call stack lives in the data segment, not here) | | |
| Program store (littlefs) | image + project source (brotli/deflate) + RETAIN | | |

The code lives in RAM so that loading never writes flash while running and the code area stays
constant per cycle. The G431 (32 KB RAM) is not a target.

Program store: its own littlefs region or the existing one from `roarm/missions.cpp`
(only the I/O thread owns it there, littlefs has no locks). **Decision E3:** one shared store
for the FCU with one owner thread (`io`), the PLC asks for flash work through a request (like
the RoArm missions). H743: mind the 128 KB sectors (every file > 512 B takes its own block).

## 5. Language scope

### 5.1 Common elements (all languages)

| Element | Stage 1 | Later |
|---|---|---|
| Elementary types | BOOL, SINT/INT/DINT/LINT, USINT/UINT/UDINT/ULINT, BYTE/WORD/DWORD/LWORD, REAL, LREAL, TIME, STRING | LTIME, DATE, TOD, DT, WSTRING, CHAR |
| Derived types | enums, structures, arrays (up to 3 dimensions, constant bounds), subranges, type alias | ARRAY[*], REF_TO, unions |
| POUs | PROGRAM, FUNCTION_BLOCK, FUNCTION | METHOD, INTERFACE, EXTENDS |
| Variables | VAR, VAR_INPUT, VAR_OUTPUT, VAR_IN_OUT, VAR_TEMP, VAR_GLOBAL, VAR_EXTERNAL, CONSTANT, AT %I/%Q/%M | RETAIN (P6), VAR_ACCESS |
| Configuration | one CONFIGURATION, one RESOURCE, one cyclic TASK, several PROGRAM instances | several cyclic tasks with their own interval and priority (P6, [§4.1](#41-iec-tasks-und-rtos-threads)), then SINGLE/event tasks |
| Literals | all of the standard (typed, `16#FF`, `T#1s500ms`, `2#1010_0101`) | |

LREAL on the H563 (Cortex-M33, single-precision FPU only) runs in software, so it is slower;
on the H743 (M7, double FPU) in hardware. The VM computes identically on both (IEEE 754).

### 5.2 ST

Full statement set: assignment, IF/ELSIF/ELSE, CASE (incl. ranges, enums),
FOR/WHILE/REPEAT, EXIT, CONTINUE, RETURN, FB calls with formal/informal parameters
(`tmr(IN := x, PT := T#1s);`, `tmr.Q`), function calls, all operators with IEC precedence,
implicit widening only where the standard allows it (otherwise explicit `*_TO_*`).

### 5.3 FBD

- Networks with boxes (functions, FB instances, operators), variables/constants, connections,
  negation of inputs/outputs, EN/ENO, connectors (named) for line continuations,
  jumps/labels/RETURN at network level.
- Execution order: networks top to bottom; inside a network by data flow (topological
  sort), ties broken by position (top before bottom, left before right); the order is shown in the
  editor (numbered boxes, like the standard's "execution order"). Feedback only explicitly via a
  variable (no implicit loops: compiler error).
- Lowering: each network becomes an AST sequence equivalent to ST (same IR, same back end),
  debug info refers to element ids instead of lines.

### 5.4 SFC

- Steps (one initial step), transitions (condition in ST as an expression or as an FBD network),
  alternative and simultaneous branches, jumps; actions in ST or FBD (or as a boolean variable).
- Action qualifiers: N, R, S, P, P0, P1, L, D, SD, DS, SL (with time where needed); step flags
  `Step.X`, `Step.T`.
- Evolution rule (E4): per cycle **one** evolution step (all transitions enabled at
  that moment fire simultaneously), no search for stable state within a cycle; this is
  documented as implementation-dependent behavior. Unsafe/unreachable nets
  (e.g. a simultaneous branch merged alternatively) are compiler errors.
- Lowering into the IR as a state machine: step flags as BOOL fields of the POU instance, action
  control (IEC action control block) as a library FB in the runtime library.

### 5.5 Standard library (Stage 1)

Type conversions (`*_TO_*`, TRUNC), numeric (ABS, SQRT, LN, LOG, EXP, SIN…ATAN, EXPT),
bit/shift (SHL, SHR, ROL, ROR, AND/OR/XOR/NOT), selection (SEL, MAX, MIN, LIMIT, MUX),
comparison (GT…NE, also on more than 2 inputs), string (LEN, LEFT, RIGHT, MID, CONCAT, INSERT, DELETE,
REPLACE, FIND), bistables SR/RS, edges R_TRIG/F_TRIG, counters CTU/CTD/CTUD, timers TP/TON/TOF.

Library POUs are written in ST and are part of the compiler (compiled with the program, only
what is used). Functions that would be slow in ST (string, math) are **natives** (VM syscall,
implemented in C++ and TS).

### 5.6 FCU library

Access to device functions that are more than "write a register" ([§6.3](#63-registerklassen-ausgang-kommando-parameter)):

| FB | Purpose |
|---|---|
| `FCU_StepperHome` | homing with a handshake (Execute/Done/Busy/Error/ErrorID, PLCopen style) over `StepperXStartHoming` |
| `FCU_StepperMoveAbsolute` | target position, Done when "in position" (status bit 3) |
| `FCU_RoArmRunMission` | start a stored RoArm mission by index (Execute/Done/Busy/Error) over the mission register (holding 110) |
| `FCU_Ws2812` | LED colors |
| `FCU_Log` | log entry into the device journal (rate-limited) |

The list is not final; it grows with the register schema.

### 5.7 Implementation-dependent parameters (Stage 1)

| Parameter | Value |
|---|---|
| Identifier length | 64 characters, case-insensitive |
| STRING default/maximum length | 80 / 254 bytes (Latin-1) |
| Array dimensions / elements per dimension | 3 / 65 535 |
| Call depth | 32 (VM error beyond that) |
| Division by zero, array index out of bounds, overflow in `*_TO_*` | runtime error → ERROR with location (no silent result) |
| Integer overflow in arithmetic | wraps around (two's complement), no error |
| SFC evolution | one evolution step per cycle (E4) |

This table is the compliance statement the standard requires; it lives in the IDE help too.

## 6. I/O mapping and control source

### 6.1 Symbolic I/O variables from the register schema

`schema/register_map/*.cs` is the single source. A new generator target (in
UniversalRegisterAccess, like `register_map.hpp`) produces `web/generated/plc-io.json`: per register
a **global I/O variable** with name, IEC type, direction, address, unit and comment, e.g.

```iecst
VAR_GLOBAL  (* generated, read-only in the IDE *)
    Valve1        AT %QX20.0 : BOOL;   (* Pneumatic valve 1, Rev1 PD10, Rev2 PA8 *)
    ConveyorPwm   AT %QW40   : UINT;   (* Conveyor motor duty [per mille] 0..1000 *)
    Lightbarrier1 AT %IX…    : BOOL;
    Tof1DistanceMm AT %IW…   : UINT;   (* [mm] *)
END_VAR
```

- Input registers → `%I`, holding registers → `%Q`. Type from the schema: `Display = "bool"` →
  BOOL, `signed` → INT/DINT, 32-bit pairs → UDINT/DINT, otherwise UINT.
- `Min/Max` from the schema: the compiler warns on constants out of range, the runtime limits
  outputs (the I/O cycle limits anyway, but the PLC output image shows the real value).
- The image contains the **hash of the I/O map**. A firmware with a different register map
  rejects the image (no wrong mapping after a firmware update) and the IDE says
  "Recompile the project".
- `%M` (marker memory): its own data area, not visible to Modbus. Exporting PLC variables to Modbus/OPC UA
  is P6 ([§13](#13-phasen)).

### 6.2 Control source switch

New schema region `Plc` (holding + input registers):

| Register | Kind | Content |
|---|---|---|
| `PlcControlSource` | input | 0 = fieldbus (Modbus/OPC UA/web UI), 1 = IEC 61131 |
| `PlcState` | input | NO_PROGRAM/STOP/RUN/HALTED/ERROR |
| `PlcCycleTimeUs`, `PlcCycleTimeMaxUs`, `PlcOverruns` | input | timing |
| `PlcProgramCrc`, `PlcErrorCode`, `PlcErrorPou`, `PlcErrorLine` | input | diagnostics |

**Who switches (decision from F2):** only the web UI (PLC page, REST `/api/plc/config`) and the
terminal (new command `plc source fieldbus|iec`, `plc run|stop|reset warm|cold`). Over Modbus/OPC UA the
mode and state are **read-only** (input registers), so a fieldbus master can never take the outputs away
from a running program, and a program never takes them from the master. For the same reason RUN/STOP/reset
are not commandable over the fieldbus.

Behavior:

- **Fieldbus mode (0)**: as today. The PLC may run (e.g. for monitoring calculations), but its output image
  is **not** written to the registers; the IDE shows that clearly ("outputs not active").
- **IEC mode (1)**: every *output register* ([§6.3](#63-registerklassen-ausgang-kommando-parameter)) belongs to the PLC.
  External writes to it are rejected:
  - Modbus TCP/RTU: exception response **0x02 ILLEGAL DATA ADDRESS** (decided, F3). A block write
    that touches even one blocked register is rejected as a whole (no partial write).
  - OPC UA: `Bad_NotWritable`; the nodes' AccessLevel stays CurrentRead|CurrentWrite (the switch
    is dynamic), the error comes on Write.
  - Web UI (register panel, pneumatics app, terminal commands `output`/`pwm`/…): rejected with
    a message; the controls show "controlled by IEC program".
  - **RoArm** (decided, F5): belongs to the switch. In IEC mode the teach mode is not
    available (`StartTeachModeRequest`, `JointJogTarget`, `CartesianJogTarget` are rejected; the
    RoArm app says why), and the mission register is a command register of the PLC
    (`FCU_RoArmRunMission`). Viewing, saving and deleting missions stays allowed (it moves nothing).
    Switching to IEC mode stops a running teach mode or mission (RoArm holds its position).
  - **Reading stays allowed everywhere**: Modbus/OPC UA see the outputs set by the PLC (monitoring).
- **Switching** (in either direction): outputs → safe state, then the new source takes over.
  IEC→fieldbus: the PLC goes to STOP. Fieldbus→IEC: RUN only if *autostart* is set or
  RUN is commanded explicitly. The switch is logged and persisted (startup mode).
- **Forcing** in debug mode overrides the PLC's output image, never the switch.

Implementation in rtoos: `RegisterBank` gets an optional **write filter** next to the `WriteHook`
(`bool permit(start, count)`, consulted under the guard before a protocol write); the Modbus server
returns the exception on `false`, the OPC UA register binding `Bad_NotWritable`. The application side
(`set_holding`) stays unfiltered. Host test in rtoos for both protocols.

### 6.3 Register classes: output, command, parameter

Today the schema does not distinguish whether a holding register is a **level output** (valve, PWM,
target position) or a **handshake** (`StepperXStartHoming`: master writes 1, `io` answers with
0/2/3). If the PLC wrote its output image to all holding registers every cycle, it would
overwrite `io`'s answers.

**Decision E5:** new schema attribute `Kind = Output | Command | Parameter` (default: Output for
ReadWrite):

| Kind | Example | In the IEC program | In IEC mode, external |
|---|---|---|---|
| Output | Valve1, ConveyorPwm, Stepper1Target | `%Q` variable, written to the register every cycle | blocked |
| Command | StepperXStartHoming, RoArm mission | only through FCU library FBs (written on an edge, the answer read back) | blocked |
| Parameter | configuration values | read-only `%I`-like variable | allowed |

Classifying every existing register is part of P2.

## 7. Image format (download)

Little-endian, 4-byte aligned:

| Section | Content |
|---|---|
| Header | magic `FCUPLC\0\1`, ISA version, image length, CRC-32 over everything, I/O map hash (32-bit), project hash (links the image ↔ debug info in the IDE), compiler version, build time |
| Task table | per IEC task: interval (Stage 1: multiple of the I/O cycle, P6: ms), priority → PLC thread, order within the thread, instruction budget, max overruns, stack size of the execution context, program entry points in call order, list of the inputs read and outputs written (task-local process image) |
| Code | bytecode of all POUs (absolute offsets in the code segment) |
| Data init | initial values of the data segment (compressed as zero runs), RETAIN area bounds |
| I/O binding | per `%I/%Q` variable: register address, width, data address, class |
| Native table | used syscall ids + expected signature hash (the VM rejects unknown/incompatible ones) |

Debug info (source mapping, variable layout, instance tree) **is not** in the image; it stays in the
project (browser), linked by the project hash. The device also keeps the **project source**
(compressed) so that any browser can reload the project from the device ("Upload from
device").

## 8. Bytecode VM (outline, detailed in its own ISA document in P1)

- **Stack machine** with typed operations (i32, i64, f32, f64; BOOL/SINT/… are widened to i32 on
  the stack, stored narrow in memory). Reason: the simplest compiler, compact code; register VM only if
  the profiling demands it.
- Segments: code (read-only), data (globals, program and FB instances, call stack frames).
  Addressing: absolute (globals), relative to `this` (FB instance), relative to the frame (locals,
  VAR_TEMP). Every load/store is checked against the segment bounds (bounds as constants per
  image, the check costs one comparison).
- Instruction groups: load/store/const, arithmetic/logic per type, comparison, conversion,
  jumps (relative, conditional), CALL/RET (functions, FB bodies with `this`), SYSCALL (natives),
  memory copy (structures, strings, arrays), `LINE`-free (the source mapping lives in the debug info,
  not in the code).
- Debug hooks: per code offset one bit in a breakpoint bitmap (code size / 8 bytes); the
  dispatch loop only checks the bitmap when debug mode is active (two loops: fast/debug).
  The VM can stop **between any two instructions** and continue; its entire state
  (PC, SP, frame chain) lives in the VM struct, not on the C stack.
- Reentrant: `Vm` (shared: code, data, natives, breakpoints) and `ExecutionContext` (per IEC
  task: PC, SP, frame chain, stack area) are separate; no static state in the VM.
- The opcode table (name, number, operands, stack effect) lives in rtoos (`rtoos/include/rtos/plc/`)
  and is the single source for: C++ dispatch, TS VM, TS assembler/disassembler, ISA documentation.
  rtoos generates the TS file with a Python tool (as in `rtoos/tools`) to `web/src/plc/generated/`;
  a test checks both sides against the same table version (ISA version in the image header).

## 9. Compiler (TypeScript)

Part of the web UI in `web/src/plc/compiler/` (decided, F6). It is DOM-free (enforced by its own
`tsconfig` without `lib: dom` for that folder), so it runs in the IDE, in vitest and in Node
(e.g. for the differential tests with the rtoos host VM).

Pipeline:

1. **Lexer/parser ST**: hand-written recursive descent with error recovery (several
   errors per run, exact positions for editor diagnostics). No GPL code (MatIEC, OpenPLC);
   the project is MIT.
2. **FBD/SFC**: the graphical models (JSON, [§11](#11-projektformat)) are turned into the AST
   ([§5.3](#53-fbd), [§5.4](#54-sfc)); positions = element ids.
3. **Semantics**: name resolution (scopes, VAR_EXTERNAL ↔ VAR_GLOBAL), types and overloads
   (ANY_NUM …), constant evaluation, checks (assignment to CONSTANT/inputs, uninitialized
   VAR_IN_OUT, unreachable code warning, SFC net checks).
4. **IR**: typed, linear three-address/stack IR per POU; optimizations kept small and safe
   (constant folding, dead code); no transform that destroys the source mapping in debug
   builds.
5. **Back end**: layout (data segment, instances, alignment), bytecode, image, debug info.
6. **Incremental**: re-analysis per POU on typing (diagnostics while typing,
   < 100 ms for 5 000 lines); full image build < 1 s.

Compiler output is deterministic (same project → byte-identical image), so tests can use
golden files.

## 10. IDE and debugging (browser)

### 10.1 Where the IDE comes from

The IDE is a page ("PLC") of the existing web UI (`web/src/plc/`, same build, same shell/router).
The web UI today is a brotli-compressed single-file app (`index.html.br`, ~514 KB
uncompressed). The IDE (editors + compiler + simulator) is put into its **own chunk** of the same
Vite build, loaded only when the PLC page is opened (`/plc-ide.js`, embedded like `index.html.br`),
so the dashboard does not get slower. To check in P2: the single-file plugin
(`vite-firmware-single-file`) must allow this one lazy chunk, and `network.cpp` gets a
second resource. Budget: ≤ 350 KB brotli. Development: `npm run dev` against a real device works as
before.

### 10.2 Functions

- Project tree: data types, POUs, global variables, generated I/O list (read-only, with
  live values), task configuration.
- ST editor: CodeMirror 6 (MIT) with highlighting, autocompletion (variables, FB members,
  standard functions with signature), live diagnostics, formatting.
- FBD and SFC editor: own SVG editors in lit (no heavy graph library): place, connect,
  automatic routing (orthogonal), execution order shown, undo/redo, copy/paste.
- Build → download → RUN in one action ("Load and start"), with a confirmation if the
  device is in IEC mode with machinery running.
- Simulation: the program runs in the TS VM in the browser; the I/O list becomes editable (inputs
  set by hand). No device needed, ideal for teaching.
- Project: autosave in IndexedDB, export/import as a file, upload/download from the device.

### 10.3 Online debugging

Over the existing WebSocket (BestBinaryBuffers, new schema file
`schema/best_binary_buffers/plc.cs`):

| Message | Direction | Content |
|---|---|---|
| `PlcAttach` | → | project hash; the device answers with state + whether the hash matches |
| `PlcWatch` | → | list (data address, type, length), at most 256 entries, replaces the old list |
| `PlcWatchValues` | ← | cycle counter + values, at most 10 Hz (end of cycle, consistent) |
| `PlcWrite` / `PlcForce` / `PlcUnforce` | → | write once / force permanently (input or output image) |
| `PlcBreakpoints` | → | list of code offsets (the IDE maps line/element → offset) |
| `PlcControl` | → | RUN, STOP, CONTINUE, STEP_INTO, STEP_OVER, STEP_OUT, warm/cold reset |
| `PlcHalted` | ← | PC, frame chain (POU instances), reason (breakpoint/step/error) |
| `PlcState` | ← | state, cycle times, error; on change + 1 Hz |

- Step granularity: ST statement, FBD box, SFC: one evolution step (stop after the transitions
  are evaluated). Step over/into/out over the frame chain.
- Variable view: instance tree (program → FB instances → members) from the debug info, values
  via `PlcWatch` for the visible entries only.
- FBD: values at the connections, BOOL as line color. SFC: active steps highlighted, step time `.T`
  displayed. ST: values inline at the end of the line (optional).
- Forced values are marked everywhere (IDE, register panel) and are logged; on STOP/switch
  every force is released.
- Several browsers may watch at the same time; only one may control (debug lease,
  is taken back on WS disconnect). Losing the connection while debugging: breakpoints and forces
  stay until another browser releases them or the program is stopped/reloaded. A halted program
  does **not** continue by itself (no unexpected start of motion); the PLC page shows it to every
  browser on connect, and the terminal can continue it too (`plc continue`).
- Several tasks (P6): a breakpoint halts **all** IEC tasks ("stop the world": the others do not
  start a new cycle, a running cycle finishes), so the variable view is consistent. Single step
  only runs the halted task.
- While HALTED: the instruction budget and overrun watchdog are suspended; time keeps running (TON fires
  after CONTINUE, as on a real PLC). The IDE warns before the first breakpoint in IEC mode
  ("outputs freeze, the I/O cycle keeps running").

### 10.4 REST

| Endpoint | Purpose |
|---|---|
| `GET /api/plc/status` | state, image info, control source |
| `PUT /api/plc/image` | upload the image (binary); check, store, load (STOP first) |
| `GET/PUT /api/plc/project` | project source on the device |
| `PUT /api/plc/config` | autostart, safe state on halt, task interval |

Large uploads go through REST, not the WebSocket. To check in P2: request body size and streaming
into the flash in the `Http::WebServer` (background work in bounded steps, rtoos network rule).

## 11. Project format

JSON (`*.fcuplc.json`), versioned:

```json
{ "format": 1, "name": "Sortieranlage", "ioMapHash": "…",
  "types": [ { "name": "E_Mode", "st": "TYPE E_Mode : (Manual, Auto); END_TYPE" } ],
  "pous":  [ { "name": "Main", "kind": "PROGRAM", "lang": "ST",  "src": "PROGRAM Main …" },
             { "name": "Seq",  "kind": "PROGRAM", "lang": "SFC", "decl": "VAR … END_VAR", "graph": { … } } ],
  "globals": "VAR_GLOBAL … END_VAR",
  "task": { "intervalCycles": 1, "programs": ["Main", "Seq"] } }
```

Declaration parts are always ST text, the bodies of FBD/SFC are graphs with stable element ids
(debug mapping, diffs). PLCopen TC6 XML import/export: P6.

## 12. Test strategy

Per the project rules (CLAUDE.md, regression tests binding):

| Level | Where | What |
|---|---|---|
| Compiler | npm package, vitest | parser (positive/negative), semantics errors, golden images, debug mapping |
| TS VM | npm package, vitest | ISA tests per opcode, standard library |
| **Differential** | rtoos host test + vitest | shared test corpus `plc-corpus/*.st` with input sequences; the TS VM and C++ VM must produce byte-identical traces (variable values per cycle) |
| C++ VM | rtoos `ctest --preset host` | ISA, bounds/budget/error stops, breakpoint/step, image checks (corrupt CRC, wrong I/O hash, unknown native) |
| Runtime on the target | rtoos self-test in QEMU + Renode (`arm`, `arm-h743`) | thread, cycle coupling, overrun detection, measured VM speed (instructions, not cycles: the emulators only count instructions) |
| Write filter | rtoos host test | Modbus exception and OPC UA Bad_NotWritable in IEC mode, reading allowed |
| IDE | `web/` vitest | editor models, FBD/SFC → AST, debug protocol (mocked WS) |
| Hardware | FCU rev 2 + rev 3 | download, RUN, switch with real valves/PWM, breakpoint with running steppers, power cycle with autostart |

Performance measurement on real hardware (DWT cycles) against the target from [§3](#3-grundsatzentscheidung-bytecode-vm-statt-nativem-maschinencode).

## 13. Phases

Each phase ends with something usable and tested; hardware verification where hardware is involved.

| Phase | Content | Done when |
|---|---|---|
| **P0** | this specification, review, open questions answered | spec "accepted" |
| **P1** | ISA document + opcode table/generator; compiler ST (types, POUs, standard library without strings); TS VM; C++ VM in rtoos (host) | the differential test corpus (≥ 50 programs) runs the same on both VMs |
| **P2** | runtime in the firmware: PLC thread (pool with 1 thread), process image, register classes (E5), write filter, control source switch incl. RoArm, image upload/storage, autostart, RUN/STOP, status registers, terminal command `plc`; minimal IDE (ST editor, build, load, state) | traffic light/sorting example in ST runs on rev 2 and rev 3; also after a power cycle without network and browser; Modbus write in IEC mode answered with 0x02 |
| **P3** | debugging: watch, write/force, breakpoints, single step, debug lease; simulation in the browser | step through an ST program on the HW, forced values in the register panel |
| **P4** | FBD: editor, lowering, live values | the same example as FBD, debuggable |
| **P5** | SFC: editor, lowering, action qualifiers, step display | sequence example (conveyor + valves + light barrier) as SFC on the HW |
| **P6** | several tasks with their own cycle time (thread pool of 3, fast I/O, `UpdateMs` in the schema), RETAIN, strings/date, FCU library complete, PLC variables → Modbus/OPC UA, PLCopen XML | as needed; several tasks first |
| **P7** (optional) | native Thumb-2 back end from the same IR, only if P2–P5 measurements show the VM is too slow | |

P1 can be developed entirely without hardware (host + browser).

## 14. Decisions from the review and postponed points

Review round 1 (2026-11-08):

| # | Question | Decision | Where in the spec |
|---|---|---|---|
| — | Where does the program run? | autonomously on the STM32; the browser only for programming/debugging | [§1](#1-ziel), [§4.3](#43-boot) |
| F1 | 50 ms as the fastest cycle? | enough for Stage 1; several IEC tasks with their own cycle time later (P6), mapped onto a thread pool (E2) instead of 1:1 | [§4.1](#41-iec-tasks-und-rtos-threads) |
| F2 | Who may switch the control source? | web UI and terminal only; read-only over Modbus/OPC UA, RUN/STOP too | [§6.2](#62-steuerquellen-umschaltung) |
| F3 | Modbus exception on a blocked write | 0x02 ILLEGAL DATA ADDRESS | [§6.2](#62-steuerquellen-umschaltung) |
| F5 | RoArm part of the switch? | yes: no teach mode in IEC mode, missions through `FCU_RoArmRunMission` | [§6.2](#62-steuerquellen-umschaltung), [§5.6](#56-fcu-bibliothek) |
| F6 | Where do the compiler/IDE live? | not a separate package: `web/src/plc/` (compiler in `web/src/plc/compiler/`), VM in rtoos | [§2](#2-gesamtarchitektur), [§9](#9-compiler-typescript) |

**Postponed (deliberately accepted for now, take up again later):**

- **[F 4] Access protection.** The web interface has no authentication; anyone on the network can load
  programs, start/stop them, force outputs and switch the control source. Accepted for Stage 1
  (lab/teaching network). Before the FCU runs in a network that is not fully trusted:
  authentication for `/api/plc/*`, the debug messages, the terminal command `plc` and the RoArm/
  register writes of the web UI (e.g. device password → session token, roles "watch" vs.
  "program/control"). The REST/WS interfaces are designed in P2 so that a token check can be added
  in one place later (one gate function for every PLC endpoint, not checks spread around).
