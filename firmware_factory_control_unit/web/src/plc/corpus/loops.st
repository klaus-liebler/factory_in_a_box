(* @cycles 5 *)
PROGRAM PLC_PRG
VAR i : INT; j : INT; s : DINT; st : INT := -2; k : INT; fib : ARRAY[0..20] OF DINT; END_VAR
FOR i := 1 TO 10 DO FOR j := i TO 10 BY 3 DO s := s + i * j; END_FOR; END_FOR;
FOR i := 9 TO 0 BY st DO s := s - i; END_FOR;
fib[0] := 0; fib[1] := 1;
FOR i := 2 TO 20 DO fib[i] := fib[i - 1] + fib[i - 2]; END_FOR;
k := 0;
REPEAT k := k + 1; IF k = 3 THEN CONTINUE; END_IF; s := s + k; UNTIL k > 6 END_REPEAT;
WHILE TRUE DO s := s + 1; IF s MOD 7 = 0 THEN EXIT; END_IF; END_WHILE;
END_PROGRAM
