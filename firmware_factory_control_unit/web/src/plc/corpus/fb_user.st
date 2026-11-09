(* @cycles 20 *)
FUNCTION_BLOCK FB_PID
VAR_INPUT sp : REAL; pv : REAL; kp : REAL := 1.0; ki : REAL := 0.1; END_VAR
VAR_OUTPUT y : REAL; END_VAR
VAR i : REAL; END_VAR
i := LIMIT(-50.0, i + ki * (sp - pv), 50.0);
y := kp * (sp - pv) + i;
END_FUNCTION_BLOCK
FUNCTION_BLOCK FB_PLANT
VAR_INPUT u : REAL; END_VAR
VAR_OUTPUT x : REAL; END_VAR
x := x + 0.2 * (u - x);
END_FUNCTION_BLOCK
PROGRAM PLC_PRG
VAR c : FB_PID; p : FB_PLANT; ctl : ARRAY[0..1] OF FB_PID; y2 : REAL; END_VAR
c(sp := 10.0, pv := p.x);
p(u := c.y);
ctl[1](sp := 5.0, pv := y2, kp := 0.5, y => y2);
END_PROGRAM
