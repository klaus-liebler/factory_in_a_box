(* @cycles 8 *)
TYPE
  S_Point : STRUCT x : INT := 1; y : INT := -1; END_STRUCT;
  S_Path : STRUCT p : ARRAY[0..3] OF S_Point; len : USINT := 4; name : DWORD := 16#CAFE; END_STRUCT;
END_TYPE
PROGRAM PLC_PRG
VAR a : S_Path; b : S_Path; m : ARRAY[1..3, 1..4] OF LREAL; i : INT := 1; j : INT := 1; k : INT; tot : LREAL; END_VAR
a.p[k].x := a.p[k].x + k; a.p[3 - k].y := a.p[k].x * 2;
b := a;
m[i, j] := INT_TO_LREAL(a.p[k].x) / 3.0;
tot := tot + m[i, j];
j := j + 1; IF j > 4 THEN j := 1; i := i + 1; END_IF;
k := (k + 1) MOD 4;
END_PROGRAM
