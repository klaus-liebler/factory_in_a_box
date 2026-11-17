(* @cycles 60 *)
(* @dt 10 *)
(* @set 5 PLC_PRG.in := 1 *)
(* @set 30 PLC_PRG.in := 0 *)
(* @set 33 PLC_PRG.in := 1 *)
(* @set 35 PLC_PRG.in := 0 *)
(* @expect PLC_PRG.ton_q_cycles = 16 *)
PROGRAM PLC_PRG
VAR in : BOOL; t1 : TON; t2 : TOF; t3 : TP; ton_q_cycles : INT; tof_q : INT; tp_q : INT; END_VAR
t1(IN := in, PT := T#90ms); t2(IN := in, PT := T#40ms); t3(IN := in, PT := T#70ms);
IF t1.Q THEN ton_q_cycles := ton_q_cycles + 1; END_IF;
IF t2.Q THEN tof_q := tof_q + 1; END_IF;
IF t3.Q THEN tp_q := tp_q + 1; END_IF;
END_PROGRAM
