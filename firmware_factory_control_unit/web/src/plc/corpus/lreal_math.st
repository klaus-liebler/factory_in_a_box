(* @cycles 20 *)
PROGRAM PLC_PRG
VAR x : LREAL := 2.0; s : LREAL; e : LREAL; a : LREAL; r : REAL := 2.0; sr : REAL; k : INT := -7; ak : INT; END_VAR
s := SQRT(x); sr := SQRT(r); x := x * 1.5 + s;
e := s * s;
a := ABS(-x); ak := ABS(k); k := k + 1;
r := r * 1.25;
END_PROGRAM
