(* @cycles 300 *)
(* @expect PLC_PRG.s = 44 *)
(* @expect PLC_PRG.u = 44 *)
PROGRAM PLC_PRG
VAR s : SINT; u : USINT; i : INT := 32700; w : UINT := 65500; d : DINT := 2147483600; l : LINT := 9223372036854775800; ul : ULINT := 18446744073709551600; END_VAR
s := s + 1; u := u + 1; i := i + 1; w := w + 1; d := d + 1; l := l + 1; ul := ul + 1;
END_PROGRAM
