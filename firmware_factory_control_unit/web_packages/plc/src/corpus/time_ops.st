(* @cycles 10 *)
(* @dt 37 *)
PROGRAM PLC_PRG
VAR t : TIME := T#1h2m3s4ms; u : TIME; n : DINT := 3; d : DINT; neg : TIME := T#-1500ms; c : BOOL; END_VAR
u := t * n - T#1d / 4 + neg;
d := TIME_TO_DINT(u);
c := u > T#0ms;
t := t / 2;
END_PROGRAM
