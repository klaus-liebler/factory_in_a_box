(* @cycles 3 *)
PROGRAM PLC_PRG
VAR a : REAL := -1.0; n1 : REAL; n2 : LREAL; inf : REAL := 3.0E38; i2 : REAL; ninf : REAL; z : REAL; c1 : BOOL; c2 : BOOL; c3 : BOOL; ab : REAL; END_VAR
n1 := SQRT(a); n2 := SQRT(REAL_TO_LREAL(a));
i2 := inf * 10.0; ninf := -i2; z := i2 + ninf;
c1 := n1 = n1; c2 := n1 <> n1; c3 := z < 0.0;
ab := ABS(ninf);
END_PROGRAM
