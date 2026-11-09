(* @cycles 50 *)
PROGRAM PLC_PRG
VAR x : REAL := 0.1; y : REAL := 3.0; s : REAL; p : REAL := 1.0; q : REAL; big : REAL := 1.0E30; tiny : REAL := 1.0E-30;
    lx : LREAL := 0.1; ls : LREAL; lp : LREAL := 1.0; c : BOOL; END_VAR
s := s + x; p := p * 1.1; q := s / y; big := big * 7.0; tiny := tiny / 3.0;
ls := ls + lx; lp := lp * 1.0000001;
c := s > q;
x := x + 0.07;
END_PROGRAM
