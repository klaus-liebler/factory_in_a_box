(* @cycles 30 *)
PROGRAM PLC_PRG
VAR a : DINT := -1000; b : DINT := 7; q : DINT; r : DINT; ua : UDINT := 4000000000; uq : UDINT; ur : UDINT;
    la : LINT := -9000000000000; lq : LINT; lr : LINT; ula : ULINT := 18000000000000000000; ulq : ULINT; si : SINT := -128; sq : SINT; END_VAR
q := a / b; r := a MOD b; uq := ua / DINT_TO_UDINT(b); ur := ua MOD 13;
lq := la / 13; lr := la MOD 13; ulq := ula / 3;
sq := si / -1;
a := a + 77; b := b + 1; ua := ua - 12345; la := la * 3;
END_PROGRAM
