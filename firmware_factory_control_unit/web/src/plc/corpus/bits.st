(* @cycles 20 *)
PROGRAM PLC_PRG
VAR b : BYTE := 16#81; w : WORD := 16#8001; d : DWORD := 16#80000001; l : LWORD := 16#8000000000000001;
    sb : BYTE; sw : WORD; rd : DWORD; rl : LWORD; x : DWORD; n : INT; m : BOOL; END_VAR
sb := SHL(b, n); sw := SHR(w, n); rd := ROL(d, n); rl := ROR(l, n);
x := (d AND 16#FF00FF00) OR (DWORD#16#00F0F0F0 XOR rd);
b := ROL(b, 3); w := NOT w; l := l XOR SHL(l, 1);
m := (b AND 16#01) = 16#01;
n := n + 1;
END_PROGRAM
