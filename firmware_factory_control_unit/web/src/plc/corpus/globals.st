(* @cycles 7 *)
VAR_GLOBAL
  g_count : UDINT := 1000;
  g_tab : ARRAY[0..4] OF INT := [5, 4, 3, 2(9)];
END_VAR
VAR_GLOBAL CONSTANT
  K : INT := 4;
END_VAR
PROGRAM PLC_PRG
VAR i : INT; mx : INT; mn : INT; mu : INT; n : INT; END_VAR
g_count := g_count * 3;
FOR i := 0 TO K DO g_tab[i] := g_tab[i] + i; END_FOR;
mx := MAX(g_tab[0], g_tab[1], g_tab[2], g_tab[3], g_tab[4]);
mn := MIN(g_tab[0], g_tab[4]);
mu := MUX(n, 11, 22, 33, 44, 55, 66, 77);
n := n + 1;
END_PROGRAM
