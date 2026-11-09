(* @cycles 10 *)
PROGRAM PLC_PRG
VAR d : DINT := 32760; i : INT; END_VAR
i := DINT_TO_INT(d);
d := d + 1;
END_PROGRAM
