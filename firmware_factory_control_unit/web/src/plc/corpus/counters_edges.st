(* @cycles 40 *)
PROGRAM PLC_PRG
VAR clk : BOOL; n : INT; up : CTU; dn : CTD; ud : CTUD; r : R_TRIG; f : F_TRIG; s : SR; rs : RS; edges : INT; END_VAR
clk := (n MOD 3) = 0;
r(CLK := clk); f(CLK := clk);
IF r.Q OR f.Q THEN edges := edges + 1; END_IF;
up(CU := clk, R := n = 25, PV := 5);
dn(CD := clk, LD := n = 0, PV := 4);
ud(CU := clk, CD := NOT clk AND (n MOD 5 = 0), R := FALSE, LD := FALSE, PV := 3);
s(S1 := n = 10, R := n = 20); rs(S := n = 12, R1 := n = 15);
n := n + 1;
END_PROGRAM
