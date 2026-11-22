// The FCU library (docs/plc-iec61131.md 5.6): function blocks for the device functions that are more
// than "write a register" - the command registers, handshakes the firmware answers in the register
// itself. Compiled with every project as a library file ("<fcu>": names with "__" are allowed there,
// only the blocks a project uses end up in the image).
//
// The command globals are bound with kind "command": the device copies the register into the variable
// before each cycle (the answer) and the variable into the register only when the program changed it
// (the start) - docs/plc-iec61131.md 6.3. The start value written is also the "busy" value of the
// register; the idle value 0 is also "done", so a block takes 0 as success only after it has seen its
// start value in the register. Error answers count at once. FCU_Ws2812SetLed needs no answer: the
// firmware sets the LED in the write itself and answers "ready" (16#FFFFFFFF), so it is Done at once. While the control source is the fieldbus
// nothing reaches the register and the block stays Busy.
import { COMMAND_VARS } from "./io-list.ts";

export const FCU_LIB_FILE = "<fcu>";

const BLOCKS = `
FUNCTION_BLOCK FCU_StepperHome
(* Sensorless homing of a stepper (StallGuard at the end stop), over StepperXStartHoming.
   Axis: 1 = stepper 1, 2 = stepper 2. A rising edge of Execute starts it; Busy until the device
   answers; then Done (position 0) or Error with ErrorID 2 = timeout (no end stop), 3 = TMC2209 does
   not answer, 16#100 = no such axis. Outputs stay while Execute is TRUE (else for one call). *)
VAR_INPUT Axis : UINT := 1; Execute : BOOL; END_VAR
VAR_OUTPUT Done : BOOL; Busy : BOOL; Error : BOOL; ErrorID : UINT; END_VAR
VAR Edge : R_TRIG; Active : UINT; Seen : BOOL; Answer : UINT; END_VAR
IF NOT Execute AND NOT Busy THEN
    Done := FALSE; Error := FALSE; ErrorID := 0;
END_IF;
Edge(CLK := Execute);
IF Edge.Q AND NOT Busy THEN
    Done := FALSE; Error := FALSE; ErrorID := 0; Seen := FALSE;
    IF Axis = 1 THEN
        __Stepper1StartHoming := 1; Active := 1; Busy := TRUE;
    ELSIF Axis = 2 THEN
        __Stepper2StartHoming := 1; Active := 2; Busy := TRUE;
    ELSE
        Error := TRUE; ErrorID := 16#100;
    END_IF;
ELSIF Busy THEN
    IF Active = 1 THEN Answer := __Stepper1StartHoming; ELSE Answer := __Stepper2StartHoming; END_IF;
    IF Answer = 1 THEN
        Seen := TRUE;
    ELSIF Answer <> 0 THEN
        Busy := FALSE; Error := TRUE; ErrorID := Answer;
    ELSIF Seen THEN
        Busy := FALSE; Done := TRUE;
    END_IF;
END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK FCU_RoArmRunMission
(* Runs a stored RoArm mission, over RoArmMissionControl. Index: the mission (1..). A rising edge of
   Execute starts it; Busy while it runs; then Done or Error with ErrorID 1 = no such mission,
   2 = teach mode active, 3 = mission not readable, 4 = another mission is running,
   16#100 = invalid index. Outputs stay while Execute is TRUE (else for one call). *)
VAR_INPUT Index : INT := 1; Execute : BOOL; END_VAR
VAR_OUTPUT Done : BOOL; Busy : BOOL; Error : BOOL; ErrorID : UINT; END_VAR
VAR Edge : R_TRIG; Active : INT; Seen : BOOL; Answer : INT; END_VAR
IF NOT Execute AND NOT Busy THEN
    Done := FALSE; Error := FALSE; ErrorID := 0;
END_IF;
Edge(CLK := Execute);
IF Edge.Q AND NOT Busy THEN
    Done := FALSE; Error := FALSE; ErrorID := 0; Seen := FALSE;
    IF Index > 0 THEN
        __RoArmMissionControl := Index; Active := Index; Busy := TRUE;
    ELSE
        Error := TRUE; ErrorID := 16#100;
    END_IF;
ELSIF Busy THEN
    Answer := __RoArmMissionControl;
    IF Answer = Active THEN
        Seen := TRUE;
    ELSIF Answer < 0 THEN
        Busy := FALSE; Error := TRUE; ErrorID := INT_TO_UINT(-Answer);
    ELSIF Answer > 0 THEN
        Busy := FALSE; Error := TRUE; ErrorID := 4;
    ELSIF Seen THEN
        Busy := FALSE; Done := TRUE;
    END_IF;
END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK FCU_MalibottiRunMission
(* Runs a mission of the Malibotti robot (it keeps them itself), over MalibottiMissionControl
   (hardware revision 3). Mission: its number (1..). A rising edge of Execute starts it; Busy while it
   runs; then Done or Error with ErrorID 1 = robot does not answer, 2 = unknown mission, 3 = aborted,
   4 = another mission is running, 5 = blocked, 6 = robot fault, 16#100 = invalid number.
   Outputs stay while Execute is TRUE (else for one call). *)
VAR_INPUT Mission : INT := 1; Execute : BOOL; END_VAR
VAR_OUTPUT Done : BOOL; Busy : BOOL; Error : BOOL; ErrorID : UINT; END_VAR
VAR Edge : R_TRIG; Active : INT; Seen : BOOL; Answer : INT; END_VAR
IF NOT Execute AND NOT Busy THEN
    Done := FALSE; Error := FALSE; ErrorID := 0;
END_IF;
Edge(CLK := Execute);
IF Edge.Q AND NOT Busy THEN
    Done := FALSE; Error := FALSE; ErrorID := 0; Seen := FALSE;
    IF Mission > 0 THEN
        __MalibottiMissionControl := Mission; Active := Mission; Busy := TRUE;
    ELSE
        Error := TRUE; ErrorID := 16#100;
    END_IF;
ELSIF Busy THEN
    Answer := __MalibottiMissionControl;
    IF Answer = Active THEN
        Seen := TRUE;
    ELSIF Answer < 0 THEN
        Busy := FALSE; Error := TRUE; ErrorID := INT_TO_UINT(-Answer);
    ELSIF Answer > 0 THEN
        Busy := FALSE; Error := TRUE; ErrorID := 4;
    ELSIF Seen THEN
        Busy := FALSE; Done := TRUE;
    END_IF;
END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK FCU_StepperMoveAbsolute
(* Moves a stepper to Position [steps], over StepperXTarget (and StepperXSpeed when Velocity > 0;
   0 keeps the speed). Axis: 1 or 2. A rising edge of Execute starts it; Busy until the axis stands
   at the target (position register = target, not moving) - then Done; CommandAborted when another
   target was written meanwhile. Error with ErrorID 1 = axis not referenced (home it first),
   2 = stepper drivers disabled (StepperEnable), 16#100 = no such axis. The firmware moves only a
   referenced, enabled axis; outputs stay while Execute is TRUE (else for one call). *)
VAR_INPUT Axis : UINT := 1; Execute : BOOL; Position : DINT; Velocity : UINT; END_VAR
VAR_OUTPUT Done : BOOL; Busy : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : UINT; END_VAR
VAR Edge : R_TRIG; Active : UINT; Goal : DINT; Status : UINT; Pos : DINT; Target : DINT; END_VAR
IF NOT Execute AND NOT Busy THEN
    Done := FALSE; CommandAborted := FALSE; Error := FALSE; ErrorID := 0;
END_IF;
Edge(CLK := Execute);
IF Axis = 1 THEN Status := Stepper1Status; ELSE Status := Stepper2Status; END_IF;
IF Edge.Q AND NOT Busy THEN
    Done := FALSE; CommandAborted := FALSE; Error := FALSE; ErrorID := 0;
    IF Axis <> 1 AND Axis <> 2 THEN
        Error := TRUE; ErrorID := 16#100;
    ELSIF NOT StepperEnable THEN
        Error := TRUE; ErrorID := 2;
    ELSIF (Status / 32) MOD 2 = 0 THEN
        Error := TRUE; ErrorID := 1;
    ELSE
        Active := Axis; Goal := Position; Busy := TRUE;
        IF Axis = 1 THEN
            Stepper1Target := Position;
            IF Velocity > 0 THEN Stepper1Speed := Velocity; END_IF;
        ELSE
            Stepper2Target := Position;
            IF Velocity > 0 THEN Stepper2Speed := Velocity; END_IF;
        END_IF;
    END_IF;
ELSIF Busy THEN
    IF Active = 1 THEN
        Status := Stepper1Status; Pos := Stepper1Position; Target := Stepper1Target;
    ELSE
        Status := Stepper2Status; Pos := Stepper2Position; Target := Stepper2Target;
    END_IF;
    IF Target <> Goal THEN
        Busy := FALSE; CommandAborted := TRUE;
    ELSIF NOT StepperEnable THEN
        Busy := FALSE; Error := TRUE; ErrorID := 2;
    ELSIF Pos = Goal AND (Status / 2) MOD 2 = 0 THEN
        Busy := FALSE; Done := TRUE;
    END_IF;
END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK FCU_Ws2812SetLed
(* Sets one WS2812 LED to a colour, over Ws2812ChXPixel (the LED index and the colour in one write).
   Channel: 1 = the strip (LED 0..7), 2 = the single LED (0). Color: RGB565 (16#F800 red, 16#07E0
   green, 16#001F blue, 16#FFFF white). A rising edge of Execute sets it: Done at once, or Error with
   ErrorID 16#100 = no such channel or LED. The other LEDs keep their colour; the channel's pattern
   register then reads 16#FFFF - a program that drives Ws2812ChXPattern itself keeps it at 16#FFFF, any
   other value is a pattern again and replaces the LEDs. One LED per channel and cycle (a second write in
   the same cycle replaces the first). Outputs stay while Execute is TRUE (else for one call). *)
VAR_INPUT Channel : UINT := 1; Index : UINT; Color : UINT; Execute : BOOL; END_VAR
VAR_OUTPUT Done : BOOL; Error : BOOL; ErrorID : UINT; END_VAR
VAR Edge : R_TRIG; END_VAR
IF NOT Execute THEN
    Done := FALSE; Error := FALSE; ErrorID := 0;
END_IF;
Edge(CLK := Execute);
IF Edge.Q THEN
    Done := FALSE; Error := FALSE; ErrorID := 0;
    IF Channel = 1 AND Index < 8 THEN
        __Ws2812Ch1Pixel := UINT_TO_UDINT(Index) * 65536 + UINT_TO_UDINT(Color); Done := TRUE;
    ELSIF Channel = 2 AND Index = 0 THEN
        __Ws2812Ch2Pixel := UINT_TO_UDINT(Color); Done := TRUE;
    ELSE
        Error := TRUE; ErrorID := 16#100;
    END_IF;
END_IF;
END_FUNCTION_BLOCK
`;

/** The library's source: the reserved command globals and the blocks. */
export function fcuLibSource(): string {
	const globals = ["VAR_GLOBAL", ...COMMAND_VARS.map((c) => `    ${c.name} : ${c.type};`), "END_VAR"];
	return `${globals.join("\n")}\n${BLOCKS}`;
}
