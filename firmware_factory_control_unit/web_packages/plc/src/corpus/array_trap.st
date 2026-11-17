(* @cycles 10 *)
PROGRAM PLC_PRG
VAR a : ARRAY[2..6] OF DINT; i : INT := 2; END_VAR
a[i] := i * 10;
i := i + 1;
END_PROGRAM
