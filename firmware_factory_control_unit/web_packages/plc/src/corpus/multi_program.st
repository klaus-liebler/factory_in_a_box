(* @cycles 6 *)
(* @programs PLC_PRG P2 *)
VAR_GLOBAL shared : DINT; END_VAR
PROGRAM PLC_PRG
VAR n : DINT; END_VAR
n := n + 1; shared := shared * 2 + n;
END_PROGRAM
PROGRAM P2
VAR last : DINT; END_VAR
last := shared; shared := shared - 1;
END_PROGRAM
