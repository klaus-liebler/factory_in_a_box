# IEC 61131-3 programming and runtime environment (specification)

Status: **review rounds 1+2 incorporated** (2026-11-08); **P1 implemented** (2026-11-09, see [§13](#13-phases)). This file is the reference:
decisions are made here first and the code follows. Questions decided in the review and points
deliberately postponed are under [§14](#14-decisions-from-the-review-and-postponed-points).

## 1. Goal

The FCU gets a PLC that is programmed **in the browser**:

- Languages: **ST** (Structured Text), **FBD** (Function Block Diagram, "FUP"), **SFC** (Sequential
  Function Chart, "AS"), following IEC 61131-3 Ed. 3.0 (subset, see [§5](#5-language-scope)).
- **Compiling in the browser** (TypeScript): source → machine-near code (bytecode, see
  [§3](#3-key-decision-bytecode-vm-instead-of-native-machine-code)) + debug information. The
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
   (differential tests, [§12](#12-test-strategy)).
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
[§13](#13-phases) P8) if the measured performance is not enough.

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
  resolution). A task not finished at its next due time counts as an overrun ([§4.2](#42-operating-states)).
- Data consistency between tasks as in the standard: no automatic locks. Single accesses up to
  32 bits are atomic; LINT/LREAL/structures are not. The compiler **warns** when a global variable
  is written in one task and read in another, and **errors out** when two tasks write the same output.
- Process image per task: a task copies at its start only the inputs it uses and writes at its end
  only its outputs (the compiler knows both per task, the image carries them in the task table).
- `KernelConfig::max_threads` grows by `plc_threads` (Stage 1: 5 → 6).

**Stage 1 (P2):** exactly one cyclic task, one PLC thread, coupled to the I/O cycle (below).
The VM context, image format (task table) and compiler are built for several tasks from
P1 on, so P7 needs no redesign.

**Several tasks with their own cycle times (P7)** additionally need fresh inputs: the
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
  Stage 1 (review F1); faster tasks come with several tasks in P7 (above).
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
`SafeValue`, default 0: valves closed, PWM 0, steppers: no new target, see [§6.3](#63-register-classes-output-command-parameter)).

### 4.3 Boot

1. Boot as today (identity, supply gate, power consumers, homing starts in `io`).
2. The PLC thread picks the active flash slot ([§4.5](#45-storage-decision-e3)) and checks the image (magic, CRC,
   ISA version, I/O map hash). The SD card is not needed for that.
3. Control source = the persisted mode. In IEC mode with *autostart* and a valid image: RUN
   at the first I/O cycle, **without any browser or network connection** (the network may even come
   up later or not at all). Without a valid image: IEC mode stays, state NO_PROGRAM/ERROR,
   outputs safe (no silent fallback to fieldbus).
4. Hardware acceptance test for this: power-cycle the device without a network cable/USB, the program
   runs (visible at the outputs and LEDs) — part of P2 ([§13](#13-phases)).

### 4.4 Resources (budget, configurable at compile time)

| | H563 (rev 1/2) | H743 (rev 3) | Today used/free |
|---|---|---|---|
| Code segment (RAM, the image is copied there) | 64 KB | 64 KB | H563 RAM: 265 KB of 576 KB |
| Data segment (globals, instances, stack) | 32 KB | 32 KB | H743 AXI RAM: 261 KB of 512 KB |
| VM in flash | ≤ 40 KB | ≤ 40 KB | flash: ~500 KB of 2 MB |
| Stack per PLC thread | ≤ 4 KB (size from the frames, rtoos rule; the IEC call stack lives in the data segment, not here) | | |
| Image (maximum) | 96 KB (code + data initial values + tables) | 96 KB | |
| PLC flash area | 2 × 128 KB at `0x081C'0000` (16 sectors of 8 KB each per slot) | 2 × 128 KB at `0x081C'0000` (the last two 128 KB sectors of bank 2) | |

The code lives in RAM so that loading never writes flash while running and the code area stays
constant per cycle. The G431 (32 KB RAM) is not a target.

### 4.5 Storage (decision E3)

Since the RoArm missions moved to the SD card (commit `5c3014a`) the FCU has no shared flash store any more,
and revision 3 no flash data area at all. Because of the autonomy requirement ([§1](#1-goal)) the
PLC has to start **without an SD card** too. So the storage is split:

| What | Where | Why |
|---|---|---|
| Image + device configuration (control source, autostart, safe state on halt) | **own internal flash area**, A/B slots | needed for the start; a missing or faulty card must not stop the PLC |
| Project source (for "Upload from device") | SD card, `/PLC/project.fcuplc.json` | large, not needed for the start |
| RETAIN variables (P7) | SD card, `/PLC/retain.a` / `retain.b` alternating, with CRC + sequence number | written often; flash sectors of 128 KB are not suited to that |

**Flash area.** The same address on both chips: `0x081C'0000`, 256 KB, at the end of bank 2. The
firmware image (today ~500 KB) stays in bank 1, so erasing and programming never stall the
running code. `rtos_add_firmware(... FLASH_DATA_START 0x081C0000)` for **all** revisions (today
rev 1/2 only). On the H563 the area today still holds the old mission littlefs, which the first SD mount
migrates and erases: before P2, either remove the migration or let it run only when the area does not
yet carry a PLC slot header (decided in P2, with a regression test).

**A/B slots, power-loss safe:**

- Each slot: header (magic, sequence number, image length, CRC) | image | configuration journal at the
  end of the slot (append-only records of one flash word each: H743 32 B, H563 16 B; the newest
  valid record wins).
- Download: the image goes into the **inactive** slot (erase, program, read back, check CRC); only
  then is the slot made active by a higher sequence number. Boot: the valid slot with the highest
  sequence number. A power loss at any point leaves the old or the new program, never half of
  one. The previous program stays in the other slot (rollback in the IDE possible, P7).
- Configuration changes (switch, autostart) append a record; a full journal is copied
  together with the image into the other slot (rare: one record per switch).
- Flow: the net thread only receives the upload into a RAM staging buffer, namely the PLC's own
  code+data segments, which are unused in STOP (hence the 96 KB maximum). Erasing and programming are done by the
  PLC thread (in STOP anyway) in bounded steps; an H743 sector erase (128 KB)
  takes a long time, the net thread must not be blocked by it (rtoos fairness rule). The upload is
  answered only after the commit or with an error.

**SD card:** FileX belongs to the `io` thread (like the missions); the PLC asks for reads/writes
through requests to `io`. Without a card: the PLC runs; "Upload from device" is not available
(the IDE says so); RETAIN variables start with their initial values, with a warning in the log and
a status bit.

## 5. Language scope

### 5.1 Common elements (all languages)

| Element | Stage 1 | Later |
|---|---|---|
| Elementary types | BOOL, SINT/INT/DINT/LINT, USINT/UINT/UDINT/ULINT, BYTE/WORD/DWORD/LWORD, REAL, LREAL, TIME, STRING | LTIME, DATE, TOD, DT, WSTRING, CHAR |
| Derived types | enums, structures, arrays (up to 3 dimensions, constant bounds), subranges, type alias | ARRAY[*], REF_TO, unions |
| POUs | in the language: PROGRAM, FUNCTION_BLOCK, FUNCTION (the standard and FCU libraries use all of them); **in the project: exactly one PROGRAM `PLC_PRG`** (P1–P3) | several user POUs of every type (P4, [§5.8](#58-pous-in-the-project-and-libraries)); METHOD, INTERFACE, EXTENDS |
| Variables | VAR, VAR_INPUT, VAR_OUTPUT, VAR_IN_OUT, VAR_TEMP, VAR_GLOBAL, VAR_EXTERNAL, CONSTANT, AT %I/%Q/%M | RETAIN (P7), VAR_ACCESS |
| Configuration | one CONFIGURATION, one RESOURCE, one cyclic TASK, several PROGRAM instances | several cyclic tasks with their own interval and priority (P7, [§4.1](#41-iec-tasks-and-rtos-threads)), then SINGLE/event tasks |
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

Access to device functions that are more than "write a register" ([§6.3](#63-register-classes-output-command-parameter)):

| FB | Purpose |
|---|---|
| `FCU_StepperHome` | homing with a handshake (Execute/Done/Busy/Error/ErrorID, PLCopen style) over `StepperXStartHoming` |
| `FCU_StepperMoveAbsolute` | target position, Done when "in position" (status bit 3) |
| `FCU_RoArmRunMission` | start a stored RoArm mission by index (Execute/Done/Busy/Error) over the mission register (holding 110) |
| `FCU_Ws2812` | LED colors |
| `FCU_Log` | log entry into the device journal (rate-limited) |

The list is not final; it grows with the register schema. Implemented (P2): `FCU_StepperHome`
(Axis, Execute → Done/Busy/Error/ErrorID: 2 timeout, 3 TMC2209 silent, 16#100 no such axis) and
`FCU_RoArmRunMission` (Index, Execute → …, ErrorID 1 no such mission, 2 teach mode, 3 unreadable,
4 another mission running), in `web/src/plc/ide/fcu-lib.ts`. They write reserved globals
(`__Stepper1StartHoming`, …; `__` names are only allowed in library files) that the image binds with
kind *command*. The idle value 0 is also the answer "done": a block takes 0 as success only after it
has seen its own start value in the register (otherwise, with the fieldbus as control source, the
start never arrives and the block would report a false Done - it stays Busy instead).

### 5.7 Implementation-dependent parameters (Stage 1)

| Parameter | Value |
|---|---|
| Identifiers | case-insensitive, any length; two underscores in a row are reserved for the libraries (`__PLC_TIME`) |
| STRING | not implemented yet (P7); planned: default/maximum length 80 / 254 bytes (Latin-1) |
| Arrays | any number of dimensions, constant DINT bounds; the size is limited by the data segment (32 KB) |
| TIME | 32-bit signed milliseconds (±24d20h31m23s647ms), resolution 1 ms; LTIME/DATE/TOD/DT: P7 |
| Untyped literals | take the type of the context (other operand, assignment target, parameter) and must fit it; without context DINT (LINT, ULINT if larger) or LREAL |
| Implicit conversions | only the widening ones of Ed. 3 table 11 (e.g. INT → DINT/REAL, BYTE → WORD); everything else needs `X_TO_Y` |
| Integer ↔ integer conversions | range checked: a value that does not fit is a runtime error |
| Bit string ↔ integer conversions | bit pattern transfer (`WORD_TO_INT(16#FFFF)` = -1), never an error; bit string ↔ REAL is not supported |
| REAL → integer | rounds half away from zero (`REAL_TO_INT(2.5)` = 3, `(-2.5)` = -3); `TRUNC` truncates and gives DINT; NaN or out of range is a runtime error |
| Arithmetic on bit strings (BYTE … LWORD) | not allowed (they are not numbers): AND, OR, XOR, NOT, SHL, SHR, ROL, ROR only |
| Enumerations | base type INT unless given; the first element is the initial value; compared with `=` and `<>` only |
| Function calls | non-formal: all inputs in order; formal (`name := value`): omitted inputs get their initial value. Function block calls are formal only |
| Global variables | visible in every POU; `VAR_EXTERNAL` is optional and checked against the global declaration |
| Recursion | a compile error; the compiler computes each task's frame area from the call graph |
| Call depth | 32 (runtime error beyond that) |
| Division by zero (also REAL/LREAL), array index out of bounds, subrange violation, `MUX` selector out of range, overflow in `*_TO_*` | runtime error → ERROR with location (no silent result) |
| Integer overflow in arithmetic | wraps around (two's complement), no error |
| Endless loop | the instruction budget per cycle (default 2 000 000) stops it: runtime error |
| `F_TRIG` | no edge at the first call when CLK is FALSE (its memory starts as TRUE) |
| SFC evolution | one evolution step per cycle (E4) |

This table is the compliance statement the standard requires; it lives in the IDE help too.

### 5.8 POUs in the project and libraries

**Stage 1 (P1–P3): one POU.** The project has exactly one program `PLC_PRG` (the name as in
CODESYS), in ST, called by the one cyclic task. Its declaration part holds its own variables;
the I/O variables come as generated globals ([§6.1](#61-symbolic-io-variables-from-the-register-schema)). It may use FBs and functions
of the standard and FCU libraries (TON, R_TRIG, `FCU_StepperHome` …), but cannot define any of its own.

The compiler, project format and debug info are still built for **several POUs** from P1 on
(the libraries already consist of FBs and functions); only the IDE allows no second POU
yet. So P4 adds no new compiler concepts, just objects in the project tree.

**P4: several POUs of different types.** In the project, any number of:

| Object | Content | Languages |
|---|---|---|
| Program (PRG) | PROGRAM, assigned to a task | ST; FBD (P5), SFC (P6) |
| Function block (FB) | FUNCTION_BLOCK, instantiated in programs/FBs | ST; FBD, SFC |
| Function (FUN) | FUNCTION with a return type | ST; FBD |
| Data type (DUT) | STRUCT, enum, alias, subrange | declaration (ST) |
| Global variable list (GVL) | VAR_GLOBAL (several lists, by topic) | declaration (ST) |
| Task configuration | tasks and their programs (Stage 1: one task, P7: several) | form |

Each POU has one implementation language, chosen when it is created and fixed afterwards (the declaration
part is always ST). Renaming updates every reference in the project.

**P7: creating and including libraries.**

- A library is a project of FBs/FUNs/DUTs/GVLs without a program, saved as a library file
  (`*.fculib.json`: name, version, author, description, source or precompiled IR + interface
  declarations, dependencies).
- Inclusion: the project's library manager lists the libraries with a fixed version; their POUs
  live in their own **namespace** (IEC 61131-3 Ed. 3, `NAMESPACE`/`USING`), e.g.
  `Conveyor.FB_Sorter`, so that two libraries never collide.
- The standard library and the FCU library become libraries in this format themselves (versioned,
  as dependencies of every project). The compiler's symbol tables have namespaces from P1 on,
  so P7 needs no redesign.
- Library sources: file import, and libraries on the device's SD card (`/PLC/LIB/`), so that
  several projects on one FCU share them.

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
  is P7 ([§13](#13-phases)).

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
- **IEC mode (1)**: every *output register* ([§6.3](#63-register-classes-output-command-parameter)) belongs to the PLC.
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

Classified (P2): `Kind = RegisterKind.Command` on `Stepper1StartHoming`, `Stepper2StartHoming`,
`RoArmMissionControl`; every other holding register is an Output (stepper speed and acceleration too:
the program sets its motion profile). There is no Parameter register yet. The generator
(UniversalRegisterAccess) passes the class on: `register_map::holding_kind()` in the firmware, `kind`
in `register-map.ts`.

Runtime rules per binding kind (`src/plc.cpp`): *input* and *parameter* register → variable before the
cycle; *output* variable → register after every cycle; *command* register → variable before the cycle
(the answer) and variable → register after it **only if the program changed the variable in this
cycle** - so a firmware answer that arrives while the program runs is never overwritten, and a start
is written exactly once. Outputs and commands only while the control source is IEC.

## 7. Image format (download)

Little-endian, 4-byte aligned:

| Section | Content |
|---|---|
| Header | magic `FCUPLC\0\1`, ISA version, image length, CRC-32 over everything, I/O map hash (32-bit), project hash (links the image ↔ debug info in the IDE), compiler version, build time |
| Task table | per IEC task: interval (Stage 1: multiple of the I/O cycle, P7: ms), priority → PLC thread, order within the thread, instruction budget, max overruns, stack size of the execution context, program entry points in call order, list of the inputs read and outputs written (task-local process image) |
| Code | bytecode of all POUs (absolute offsets in the code segment) |
| Data init | initial values of the data segment (compressed as zero runs), RETAIN area bounds |
| I/O binding | per `%I/%Q` variable: register address, width, data address, class |
| Native table | used syscall ids + expected signature hash (the VM rejects unknown/incompatible ones) |

Debug info (source mapping, variable layout, instance tree) **is not** in the image; it stays in the
project (browser), linked by the project hash. The device also keeps the **project source**
on the SD card ([§4.5](#45-storage-decision-e3)) so that any browser can reload the project from the device ("Upload from
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
2. **FBD/SFC**: the graphical models (JSON, [§11](#11-project-format)) are turned into the AST
   ([§5.3](#53-fbd), [§5.4](#54-sfc)); positions = element ids.
3. **Semantics**: name resolution (scopes, VAR_EXTERNAL ↔ VAR_GLOBAL), types and overloads
   (ANY_NUM …), constant evaluation, checks (assignment to CONSTANT/inputs, uninitialized
   VAR_IN_OUT, unreachable code warning, SFC net checks).
4. **IR**: the type checker produces a typed expression tree with every implicit conversion
   explicit; the back end emits bytecode from it through an assembler with labels (P1). A
   separate linear IR comes only when a back end needs it (native code, P8). Optimizations stay
   small and safe; none may destroy the source mapping of the debugger.
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

### 10.2 Interface (layout and workflows)

**How much the spec fixes:** structure, object model, workflows and online behavior, because they
affect the protocol, the firmware and the tests. Not fixed: look (colors, spacing, icons),
which follows the existing web UI. Before P2 there is a clickable mockup for review; this section is
then corrected rather than re-specified.

**Models:** CODESYS (the de facto standard for IEC 61131-3 environments, familiar to many users) and the
[OpenPLC Editor](https://github.com/Autonomy-Logic/openplc-editor) (a modern web-technology IDE).
OpenPLC is GPLv3: we borrow **concepts** only, never code (the FCU web UI is MIT). Its structure shows
the same building blocks as CODESYS: explorer/project tree, a dialog to create objects (type +
language + name), a variable table with a table/text switch, task and instance tables, a library tree,
console/PLC log, a debugger with a variable tree and value badges in the graphical editors, cycle time
statistics.

**Layout** (desktop, from 1280 px wide):

```
┌──────────────────────────────────────────────────────────────────────────────────────┐
│ Toolbar: Build │ Connect │ Download │ ▶ Run ■ Stop ⟲ Reset │ Debug: ⏵ ⤼ ⤵ ⤴ │ Simulation │
├───────────────┬──────────────────────────────────────────────────────────────────────┤
│ Project tree  │ [PLC_PRG ●] [FB_Sorter] [GVL_Machine] [I/O]        tabs, ● = modified │
│               │ ┌──────────────────────────────────────────────────────────────────┐ │
│ ▾ FCU (online)│ │ declaration part      [table | text]    online: value │ prepared  │ │
│   ▾ App       │ ├────────────────────────── divider ───────────────────────────────┤ │
│     PLC_PRG   │ │ implementation (ST / FBD / SFC)                                  │ │
│     POUs …    │ │   online: inline values, active SFC steps, BOOL lines in color   │ │
│     DUTs …    │ └──────────────────────────────────────────────────────────────────┘ │
│     GVLs …    ├──────────────────────────────────────────────────────────────────────┤
│     Tasks     │ Panel: Messages │ Watch 1–4 │ Call stack │ Device log │ Cross refs    │
│   I/O (gen.)  │                                                                      │
│   Libraries   │                                                                      │
├───────────────┴──────────────────────────────────────────────────────────────────────┤
│ Status: ● online │ RUN │ source: IEC │ cycle 0.4/0.9 ms │ ⚠ project ≠ device │ 2 forced │
└──────────────────────────────────────────────────────────────────────────────────────┘
```

| Element | Behavior (borrowed from) |
|---|---|
| Project tree | device → application → objects; context menu "Add object …" (both). In P1–P3 only `PLC_PRG`, I/O and libraries (read-only) |
| "Add object" dialog | type (PRG/FB/FUN/DUT/GVL), name (checked for validity and uniqueness straight away), language; FUN also return type (both) |
| POU editor | declaration part above, implementation below, divider adjustable (CODESYS). The declaration part can be switched between **table** (name, class, type, initial value, comment) and **text**; both edit the same ST text (CODESYS tabular declaration, OpenPLC variables table + view mode toggle) |
| ST editor | CodeMirror 6 (MIT): highlighting, autocompletion (variables, FB members, standard functions with signature), live diagnostics, formatting, folding |
| FBD/SFC editors | own SVG editors in lit (no heavy graph library): palette, drag & drop, orthogonal routing, execution order shown, undo/redo, copy/paste, zoom |
| I/O view | the generated I/O variables as a table: name, address, type, unit, update period, comment; online with live values (CODESYS I/O mapping) |
| Messages | build errors/warnings, a click jumps to the place (both) |
| Watch lists | up to 4, variables via context menu/drag, freely named (CODESYS watch lists) |
| Device log | the PLC entries of the device journal (ERROR with location, switches, downloads) |
| Status bar | connection, PLC state with color, control source, cycle time current/max, project-vs-device indicator, number of forced values (CODESYS status bar) |

**Online workflow** (CODESYS "Login"):

1. *Connect*: the IDE compares the project hash with the image on the device.
   - Same: online, monitoring straight away.
   - Different: dialog "Download the project (PLC goes to STOP)" / "Upload the project from the device" /
     "Cancel". (No online change; P7 at the earliest.)
2. Online the editors show values: ST **inline** behind every variable (CODESYS inline
   monitoring), the declaration table gets the "current value" and "prepared value" columns.
3. **Prepared values** (CODESYS): enter values for several variables, then *Write*
   (once) or *Force* (permanently); all prepared values go out together and take effect in the **same**
   cycle. Forced values are marked in color everywhere; "release every force" in the toolbar.
4. Opening an FB online asks which instance (CODESYS); the watch view shows instances as a
   tree (OpenPLC debugger tree).
5. Debug: set a breakpoint per line/FBD box (gutter click), continue, step into, step over,
   step out, run to cursor. The call stack panel shows the frame chain; a click jumps there.

**Other things:**

- *Load and start*: build → download → RUN in one action, with a confirmation when the device is in IEC
  mode and RUN (machinery may move).
- *Simulation*: the program runs in the TS VM in the browser; the I/O view becomes editable (set inputs
  by hand), outputs are only shown. Same operation as online; no device needed, ideal
  for teaching.
- Project: autosave in IndexedDB, export/import as a file, upload/download from the device.
- Keyboard shortcuts based on CODESYS (F9 breakpoint, F10 step over, F8 step into …) where the
  browser allows it; keys the browser keeps or that collide (F5 reload, F11
  full screen, Shift+F10 context menu …) get alternatives. The final assignment is tested in the
  browsers in P2/P3 and documented in the IDE help.
- Narrow screens (phone): no editing; status, RUN/STOP and watch lists only (read-only).

Not borrowed: LD and IL, cloud/source-control functions, the OpenPLC hardware/fieldbus configuration
(the I/O comes from the register schema at the FCU).

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
- Several tasks (P7): a breakpoint halts **all** IEC tasks ("stop the world": the others do not
  start a new cycle, a running cycle finishes), so the variable view is consistent. Single step
  only runs the halted task.
- While HALTED: the instruction budget and overrun watchdog are suspended; time keeps running (TON fires
  after CONTINUE, as on a real PLC). The IDE warns before the first breakpoint in IEC mode
  ("outputs freeze, the I/O cycle keeps running").

### 10.4 Device interface (WebSocket)

The web UI talks to the firmware over binary WebSockets only (project decision: the SPA once, then
`wss://<host>/ws` with BestBinaryBuffers); the PLC uses its own namespace `plc`
(`schema/best_binary_buffers/plc.cs`) instead of the REST endpoints planned here before:

| Message | Answered by | Purpose |
|---|---|---|
| `StatusRequest` | network thread | state, control source, autostart, cycles, instructions, image and its project hash, trap, why downloads are refused |
| `ControlRequest` | PLC thread | RUN, STOP, warm/cold reset, control source IEC/fieldbus, autostart on/off (stored in the flash slot's journal) |
| `UploadRequest` | PLC thread | a chunk of up to 1 KB of the image into the device's RAM (offset 0 starts; the PLC must not be in RUN) |
| `CommitRequest` | PLC thread | CRC, VM verifier, I/O bindings against the register map, then the flash slot (A/B); answered when the flash is written |
| `ReadRequest` | network thread | up to 1 KB of the data segment (online values) |
| `WriteRequest` | PLC thread | up to 64 bytes into the data segment between two cycles (prepared values) |

The chunks keep every message under the server's 1280 bytes; flash work runs in the PLC thread, so the
network thread never waits for an erase (rtoos fairness rule). Project upload/download from the SD card
(`GetProject`/`PutProject`) follows with the IDE's device mode.

**I/O check at the commit:** the image's `ioMapHash` must equal `register_map::map_hash`, a CRC-32 the
register generator computes over the map's layout (bank, address, width, name, signedness, bool, class
of every register; descriptions, units and limits do not count) and also writes to `register-map.ts`
(`REGISTER_MAP_HASH`), which the IDE puts into the image. A firmware with another register map refuses
the image ("recompile the project"). Each binding is checked as well: register range and width, and the
register class (an output binding only on Output registers, a command binding only on Command
registers, ...).

## 11. Project format

JSON (`*.fcuplc.json`), versioned:

Stage 1 (P1–P3), exactly one program:

```json
{ "format": 1, "name": "Ampel", "ioMapHash": "…",
  "libraries": [ { "name": "Standard", "version": "1.0.0" }, { "name": "FCU", "version": "1.0.0" } ],
  "objects": [ { "id": "…", "name": "PLC_PRG", "kind": "PRG", "lang": "ST",
                 "decl": "PROGRAM PLC_PRG VAR … END_VAR", "body": "…" } ],
  "tasks": [ { "name": "MainTask", "intervalCycles": 1, "programs": ["PLC_PRG"] } ] }
```

From P4 `objects` holds any number of entries of every type (`PRG`, `FB`, `FUN`, `DUT`, `GVL`),
grouped in folders (`"folder": "Sortierung"`), `body` for FBD/SFC is a graph with stable element
ids (debug mapping, diffs). The format already has `objects`, `libraries` and `tasks` as lists in
Stage 1, so P4/P7 need no format change. A library file (`*.fculib.json`, P7) has
the same structure without `tasks`, plus `version`, `namespace` and `dependencies`. PLCopen TC6 XML
import/export: P7.

## 12. Test strategy

Per the project rules (CLAUDE.md, regression tests binding):

| Level | Where | What |
|---|---|---|
| Compiler | `web/src/plc/compiler`, vitest | parser (positive/negative), semantics errors, golden images, debug mapping |
| TS VM | `web/src/plc`, vitest | ISA tests per opcode, standard library |
| **Differential** | rtoos host test + vitest | shared test corpus `plc-corpus/*.st` with input sequences; the TS VM and C++ VM must produce byte-identical traces (variable values per cycle) |
| C++ VM | rtoos `ctest --preset host` | ISA, bounds/budget/error stops, breakpoint/step, image checks (corrupt CRC, wrong I/O hash, unknown native) |
| Runtime on the target | rtoos self-test in QEMU + Renode (`arm`, `arm-h743`) | thread, cycle coupling, overrun detection, measured VM speed (instructions, not cycles: the emulators only count instructions) |
| Write filter | rtoos host test | Modbus exception and OPC UA Bad_NotWritable in IEC mode, reading allowed |
| IDE | `web/` vitest | editor models, FBD/SFC → AST, debug protocol (mocked WS) |
| Hardware | FCU rev 2 + rev 3 | download, RUN, switch with real valves/PWM, breakpoint with running steppers, power cycle with autostart |

Performance measurement on real hardware (DWT cycles) against the target from [§3](#3-key-decision-bytecode-vm-instead-of-native-machine-code).

## 13. Phases

Each phase ends with something usable and tested; hardware verification where hardware is involved.

| Phase | Content | Done when |
|---|---|---|
| **P0** | this specification, review, open questions answered | spec "accepted" |
| **P1** | ISA document + opcode table/generator; compiler ST (types, `PLC_PRG` + standard library without strings, internally already multi-POU with namespaces); TS VM; C++ VM in rtoos (host) | the differential test corpus (≥ 50 programs) runs the same on both VMs |
| **P2** | runtime in the firmware: PLC thread (pool with 1 thread), process image, register classes (E5), write filter, control source switch incl. RoArm, flash A/B slots + project on the SD card (E3), autostart, RUN/STOP, status registers, terminal command `plc`; clickable GUI mockup reviewed; minimal IDE (project tree, `PLC_PRG` in ST, build, connect, download, state) | traffic light/sorting example in ST runs on rev 2 and rev 3; also after a power cycle without network, browser **and SD card**; power cut during the download leaves the old program; Modbus write in IEC mode answered with 0x02 |
| **P3** | debugging: online workflow, inline values, watch lists, prepared values write/force, breakpoints, single step, call stack, debug lease; simulation in the browser | step through `PLC_PRG` on the HW, forced values in the register panel |
| **P4** | several POUs: PRG/FB/FUN/DUT/GVL in the project, "Add object" dialog, declaration table ↔ text, renaming with references, cross references, FB instance selection online | sorting example split into FB + PRG + GVL, debuggable |
| **P5** | FBD: editor, lowering, live values | an FB of the example as FBD, debuggable |
| **P6** | SFC: editor, lowering, action qualifiers, step display | sequence example (conveyor + valves + light barrier) as SFC on the HW |
| **P7** | libraries (create, version, include, namespaces, standard/FCU library as libraries); several tasks with their own cycle time (thread pool of 3, fast I/O, `UpdateMs` in the schema); RETAIN (SD card); strings/date; FCU library complete; PLC variables → Modbus/OPC UA; PLCopen XML; rollback to the previous slot | as needed |
| **P8** (optional) | native Thumb-2 back end from the same IR, only if P2–P6 measurements show the VM is too slow | |

P1 can be developed entirely without hardware (host + browser).

**P1 result (2026-11-09).** Instruction set: `rtoos/tools/plc_isa.py` (155 opcodes), documented with the image format and
the verifier in `rtoos/docs/plc.md`. C++ VM: `rtoos/src/plc/vm.cpp` (~13.5 KB on the Cortex-M33/M7). TypeScript:
`web/src/plc/` (compiler in `compiler/`, VM `vm.ts`, image writer `image.ts`). Tests: 29 C++ and 26 TypeScript VM tests
with the same cases, 37 compiler tests, and the differential corpus of 66 programs (26 written by hand in
`web/src/plc/corpus/`, 40 from a seeded random generator) whose traces the C++ VM reproduces byte for byte
(`npm run plc:corpus` regenerates `rtoos/tests/plc_corpus/`). A deliberately broken rounding in the C++ VM makes the
corpus fail, so it does detect differences. Open from P1: the compiler's symbol tables have no namespaces yet
(needed for libraries, P7); STRING is not implemented (P7).

**P2 state (2026-11-09, without hardware).** Done and tested in Renode on revision 3 (STM32H743) and 2
(STM32H563), `tools/test-plc-renode.py`: PLC thread coupled to the I/O cycle, I/O bindings in the image,
A/B flash slots with configuration journal (power loss tested on the host for both geometries, the slots in
the emulated flash of both chips), control source with write filter (Modbus 0x02, OPC UA Bad_NotWritable, web
UI, terminal, RoArm teach), safe state, autostart after a reset without a browser, the refusal while the old
mission store occupies the area. The IDE has the FCU as a second device (download, RUN/STOP, control source,
autostart, online values); `tools/run-renode-fcu.py` runs the firmware in Renode to try it.
Since then (same day): register classes in the schema (E5) with the binding kind *command* in the runtime,
the firmware's check of `ioMapHash` against the generated `register_map::map_hash`, the FCU library blocks
`FCU_StepperHome` and `FCU_RoArmRunMission` (simulated in the IDE's plant model). In IEC mode the command
registers now belong to the program too (Modbus 0x02; the terminal's `home` refuses). Renode
(`tools/test-plc-renode.py`, both revisions): an image for another register map is refused; the homing
started by `FCU_StepperHome` reaches the firmware once, the block gets its answer (no TMC2209 in the
emulator: Error 3). The terminal command `plc`: without arguments the state (read directly), `plc run |
stop | reset warm|cold | source fieldbus|iec | autostart on|off` go to the PLC thread as a request -
they may write the flash journal, the I/O thread that runs the terminal does not wait for that - and the
PLC thread logs the result (Renode: state, refused arguments, source and autostart through the log).
Still open in P2: the project source on the SD card ("upload from device"), the other FCU blocks (`FCU_StepperMoveAbsolute`, `FCU_Ws2812`, `FCU_Log`), and
the hardware test on the boards.

## 14. Decisions from the review and postponed points

Review round 1 (2026-11-08):

| # | Question | Decision | Where in the spec |
|---|---|---|---|
| — | Where does the program run? | autonomously on the STM32; the browser only for programming/debugging | [§1](#1-goal), [§4.3](#43-boot) |
| F1 | 50 ms as the fastest cycle? | enough for Stage 1; several IEC tasks with their own cycle time later (P7), mapped onto a thread pool (E2) instead of 1:1 | [§4.1](#41-iec-tasks-and-rtos-threads) |
| F2 | Who may switch the control source? | web UI and terminal only; read-only over Modbus/OPC UA, RUN/STOP too | [§6.2](#62-control-source-switch) |
| F3 | Modbus exception on a blocked write | 0x02 ILLEGAL DATA ADDRESS | [§6.2](#62-control-source-switch) |
| F5 | RoArm part of the switch? | yes: no teach mode in IEC mode, missions through `FCU_RoArmRunMission` | [§6.2](#62-control-source-switch), [§5.6](#56-fcu-library) |
| F6 | Where do the compiler/IDE live? | not a separate package: `web/src/plc/` (compiler in `web/src/plc/compiler/`), VM in rtoos | [§2](#2-overall-architecture), [§9](#9-compiler-typescript) |

Review round 2 (2026-11-08):

| # | Question | Decision | Where in the spec |
|---|---|---|---|
| E3 | Storage after the missions moved to the SD card | image + configuration in their own flash area (A/B slots at `0x081C'0000`, both chips), project source and RETAIN on the SD card; the PLC starts without a card | [§4.5](#45-storage-decision-e3) |
| — | Several POUs | several POUs of every type (P4); until then exactly one `PLC_PRG`; compiler/format built for several from P1 on | [§5.8](#58-pous-in-the-project-and-libraries) |
| — | Libraries | create and include (P7), versioned, with namespaces | [§5.8](#58-pous-in-the-project-and-libraries) |
| — | How much GUI in the spec? | structure, object model, workflows, online behavior after CODESYS/OpenPLC; look via a mockup before P2 | [§10.2](#102-interface-layout-and-workflows) |

**Postponed (deliberately accepted for now, take up again later):**

- **[F 4] Access protection.** The web interface has no authentication; anyone on the network can load
  programs, start/stop them, force outputs and switch the control source. Accepted for Stage 1
  (lab/teaching network). Before the FCU runs in a network that is not fully trusted:
  authentication for `/api/plc/*`, the debug messages, the terminal command `plc` and the RoArm/
  register writes of the web UI (e.g. device password → session token, roles "watch" vs.
  "program/control"). The REST/WS interfaces are designed in P2 so that a token check can be added
  in one place later (one gate function for every PLC endpoint, not checks spread around).
