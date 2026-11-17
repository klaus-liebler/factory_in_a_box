(* @cycles 30 *)
(* @dt 25 *)
FUNCTION_BLOCK FB_BLINK
VAR_INPUT en : BOOL; period : TIME := T#100ms; END_VAR
VAR_OUTPUT q : BOOL; toggles : INT; END_VAR
VAR t : TON; END_VAR
t(IN := en AND NOT t.Q, PT := period / 2);
IF t.Q THEN q := NOT q; toggles := toggles + 1; END_IF;
END_FUNCTION_BLOCK
PROGRAM PLC_PRG
VAR b1 : FB_BLINK; b2 : FB_BLINK; both : INT; END_VAR
b1(en := TRUE);
b2(en := TRUE, period := T#250ms);
IF b1.q AND b2.q THEN both := both + 1; END_IF;
END_PROGRAM
