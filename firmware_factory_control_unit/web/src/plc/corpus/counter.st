(* @cycles 10 *)
(* @expect PLC_PRG.n = 10 *)
PROGRAM PLC_PRG
VAR n : DINT; END_VAR
n := n + 1;
END_PROGRAM
