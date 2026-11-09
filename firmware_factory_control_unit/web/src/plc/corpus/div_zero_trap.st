(* @cycles 10 *)
PROGRAM PLC_PRG
VAR a : DINT := 100; b : DINT := 4; c : DINT; END_VAR
c := a / b;
b := b - 1;
END_PROGRAM
