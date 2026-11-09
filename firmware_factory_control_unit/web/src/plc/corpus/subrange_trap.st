(* @cycles 20 *)
TYPE T_Pct : INT (0..100); END_TYPE
PROGRAM PLC_PRG
VAR p : T_Pct := 40; END_VAR
p := p + 7;
END_PROGRAM
