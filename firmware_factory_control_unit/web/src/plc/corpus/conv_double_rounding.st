(* @cycles 2 *)
PROGRAM PLC_PRG
VAR v : LINT := 1152921573326323713; f : REAL; g : REAL; uv : ULINT := 18446744073709551615; h : REAL; hl : LREAL; END_VAR
(* 2^60 + 2^36 + 1: rounded once to REAL gives 2^60 + 2^37 *)
f := LINT_TO_REAL(v); g := LINT_TO_REAL(-v); h := ULINT_TO_REAL(uv); hl := ULINT_TO_LREAL(uv);
v := v + 1;
END_PROGRAM
