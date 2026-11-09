(* @cycles 40 *)
PROGRAM PLC_PRG
VAR i : INT := -20; u : UDINT := 4294967270; l : LINT := -5; ul : ULINT := 18446744073709551590;
    c1 : BOOL; c2 : BOOL; c3 : BOOL; c4 : BOOL; c5 : BOOL; c6 : BOOL; n : DINT; END_VAR
c1 := i < 0; c2 := u > 4294967282; c3 := l <= -3; c4 := ul >= 18446744073709551602; c5 := i <> 0; c6 := (i = 0) OR (u = 0);
IF c1 THEN n := n + 1; END_IF;
IF c2 AND c4 THEN n := n + 100; END_IF;
i := i + 1; u := u + 1; l := l + 1; ul := ul + 1;
END_PROGRAM
