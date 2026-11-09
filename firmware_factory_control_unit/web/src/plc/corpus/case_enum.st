(* @cycles 12 *)
TYPE E_State : (Idle, Fill, Heat := 10, Drain, Done); END_TYPE
PROGRAM PLC_PRG
VAR st : E_State; t : INT; n : INT; hist : ARRAY[0..11] OF E_State; c : INT; END_VAR
CASE st OF
  Idle: st := Fill;
  Fill: t := t + 1; IF t >= 3 THEN st := Heat; t := 0; END_IF;
  Heat, Drain: t := t + 2; IF t > 3 THEN st := SEL(st = Heat, Done, Drain); t := 0; END_IF;
ELSE
  st := Idle;
END_CASE;
CASE n OF
  0..2: c := 1;
  3, 5, 7: c := 2;
  -5..-1: c := 3;
ELSE c := 4;
END_CASE;
hist[n] := st;
n := n + 1;
END_PROGRAM
