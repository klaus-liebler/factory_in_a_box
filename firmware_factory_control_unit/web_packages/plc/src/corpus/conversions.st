(* @cycles 25 *)
PROGRAM PLC_PRG
VAR r : REAL := -12.5; lr : LREAL := 1.0E15; i : INT; d : DINT; l : LINT; ul : ULINT; t : DINT;
    rr : REAL; lrr : LREAL; b : BOOL; w : WORD; dw : DWORD; si : SINT; k : DINT := -3; END_VAR
i := REAL_TO_INT(r); d := TRUNC(r); l := LREAL_TO_LINT(lr); t := REAL_TO_DINT(r * 1000.0);
rr := DINT_TO_REAL(k * 33333); lrr := LINT_TO_LREAL(l * 3);
b := INT_TO_BOOL(i); w := INT_TO_WORD(i); dw := WORD_TO_DWORD(w); si := WORD_TO_SINT(w);
ul := LINT_TO_ULINT(ABS(l));
r := r + 1.0; lr := lr * 3.0; k := k * 7;
END_PROGRAM
