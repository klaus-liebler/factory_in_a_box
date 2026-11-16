// The VDI 3814 library (building automation, GA functions of VDI 3814 Blatt 4.3:2022, the columns of the
// GA-Funktionsliste): group 1 input/output functions and group 2 application functions as function blocks
// and functions in ST, in the namespace VDI3814 (`VDI3814.PID`, or `USING VDI3814;`). Group 3 (operating,
// display and management: graphics, dynamisation, history) is the HMI (ide/hmi*.ts).
//
// Compiled with every project like the FCU library (only what a project uses ends up in the image). The
// first line of a block's comment is its title in the FBD palette. Times between two calls come from the
// cycle's start time (__PLC_TIME), so every block works at any task interval; the first call of an
// instance sees no time step. Units: temperatures in °C, positions and controller outputs in %, times
// as TIME unless a comment says otherwise ([s]: REAL seconds).
//
// The standard timers TP, TON, TOF cover "2.1 Zeiten"; the PLC has no wall clock yet (DATE/TOD: P7), so
// VDI3814.CLOCK keeps one that the operator sets (HMI) for the schedule and the calendar.

export const VDI3814_LIB_FILE = "<vdi3814>";
export const VDI3814_NAMESPACE = "VDI3814";

interface Section {
	/** The palette group. */
	group: string;
	source: string;
}

/** Internal helpers (names with "__" are not offered as boxes). */
const HELPERS = `
FUNCTION_BLOCK __Dt
(* Zeitschritt seit dem letzten Aufruf *)
VAR_OUTPUT Dt : REAL; (* [s], 0 at the first call, at most 1 s (after a halt) *) DtT : TIME; END_VAR
VAR Last : TIME; Init : BOOL; Now : TIME; END_VAR
Now := __PLC_TIME();
IF Init THEN
    DtT := Now - Last;
    IF DtT > T#1s THEN DtT := T#1s; END_IF;
    IF DtT < T#0s THEN DtT := T#0s; END_IF;
    Dt := TIME_TO_REAL(DtT) * 0.001;
ELSE
    Dt := 0.0; DtT := T#0s; Init := TRUE;
END_IF;
Last := Now;
END_FUNCTION_BLOCK

FUNCTION __Within : BOOL
(* Liegt Now im Zeitfenster [OnT, OffT)? OffT < OnT: über Mitternacht; OnT = OffT: nie *)
VAR_INPUT Now : TIME; OnT : TIME; OffT : TIME; END_VAR
IF OnT = OffT THEN
    __Within := FALSE;
ELSIF OnT < OffT THEN
    __Within := Now >= OnT AND Now < OffT;
ELSE
    __Within := Now >= OnT OR Now < OffT;
END_IF;
END_FUNCTION

FUNCTION __Days : DINT
(* Tage seit 1970-01-01 (proleptischer gregorianischer Kalender) *)
VAR_INPUT Y : INT; M : INT; D : INT; END_VAR
VAR Yy : DINT; Era : DINT; Yoe : DINT; Doy : DINT; Mm : DINT; END_VAR
Yy := Y;
IF M <= 2 THEN Yy := Yy - 1; END_IF;
IF Yy >= 0 THEN Era := Yy / 400; ELSE Era := (Yy - 399) / 400; END_IF;
Yoe := Yy - Era * 400;
IF M > 2 THEN Mm := M - 3; ELSE Mm := M + 9; END_IF;
Doy := (153 * Mm + 2) / 5 + D - 1;
__Days := Era * 146097 + Yoe * 365 + Yoe / 4 - Yoe / 100 + Doy - 719468;
END_FUNCTION

FUNCTION __Atan2 : REAL
(* Winkel von (X, Y) in rad, -pi..pi *)
VAR_INPUT Y : REAL; X : REAL; END_VAR
IF X > 0.0 THEN
    __Atan2 := ATAN(Y / X);
ELSIF X < 0.0 AND Y >= 0.0 THEN
    __Atan2 := ATAN(Y / X) + 3.14159265;
ELSIF X < 0.0 THEN
    __Atan2 := ATAN(Y / X) - 3.14159265;
ELSIF Y > 0.0 THEN
    __Atan2 := 1.57079633;
ELSIF Y < 0.0 THEN
    __Atan2 := -1.57079633;
ELSE
    __Atan2 := 0.0;
END_IF;
END_FUNCTION
`;

const SECTIONS: Section[] = [
	{
		group: "VDI 3814 · 1.1 E/A physikalisch",
		source: `
FUNCTION_BLOCK AI
(* Analoge Eingabe (AI): Rohwert skalieren, filtern, Messbereich überwachen
   Raw is scaled linearly from RawLo..RawHi to EngLo..EngHi, Offset added, then filtered (PT1, Filter [s],
   0 = none). A raw value outside FaultLo..FaultHi (wire break, short circuit) sets Fault and holds the
   last good value. OutOfService: PV = Manual (the object is "out of service"). *)
VAR_INPUT
    Raw : REAL;
    RawLo : REAL := 0.0; RawHi : REAL := 4095.0;
    EngLo : REAL := 0.0; EngHi : REAL := 100.0;
    Offset : REAL;
    Filter : REAL;
    FaultLo : REAL := -1.0E30; FaultHi : REAL := 1.0E30;
    OutOfService : BOOL; Manual : REAL;
END_VAR
VAR_OUTPUT PV : REAL; Fault : BOOL; END_VAR
VAR T : __Dt; F : REAL; X : REAL; Init : BOOL; END_VAR
T();
Fault := Raw < FaultLo OR Raw > FaultHi;
IF RawHi <> RawLo THEN
    X := EngLo + (Raw - RawLo) * (EngHi - EngLo) / (RawHi - RawLo) + Offset;
ELSE
    X := EngLo + Offset;
END_IF;
IF NOT Fault THEN
    IF NOT Init OR Filter <= 0.0 THEN
        F := X; Init := TRUE;
    ELSE
        F := F + (X - F) * (1.0 - EXP(-T.Dt / Filter));
    END_IF;
END_IF;
IF OutOfService THEN PV := Manual; ELSIF Init THEN PV := F; END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK BI
(* Binäre Eingabe (BI): Polarität, Entprellung, Schaltspiele und Einschaltdauer
   PV = In XOR Invert after it was stable for Debounce. Changes counts inactive -> active,
   ActiveTime the hours PV was TRUE; ResetCount clears both. OutOfService: PV = Manual. *)
VAR_INPUT In : BOOL; Invert : BOOL; Debounce : TIME; OutOfService : BOOL; Manual : BOOL; ResetCount : BOOL; END_VAR
VAR_OUTPUT PV : BOOL; Changes : UDINT; ActiveTime : LREAL; (* [h] *) END_VAR
VAR T : __Dt; Cand : BOOL; F : BOOL; Stable : TIME; Last : BOOL; END_VAR
T();
IF (In XOR Invert) <> Cand THEN
    Cand := In XOR Invert; Stable := T#0s;
ELSIF Stable < Debounce THEN
    Stable := Stable + T.DtT;
END_IF;
IF Stable >= Debounce THEN F := Cand; END_IF;
IF OutOfService THEN PV := Manual; ELSE PV := F; END_IF;
IF ResetCount THEN
    Changes := 0; ActiveTime := 0.0;
ELSE
    IF PV AND NOT Last THEN Changes := Changes + 1; END_IF;
    IF PV THEN ActiveTime := ActiveTime + T.Dt / 3600.0; END_IF;
END_IF;
Last := PV;
END_FUNCTION_BLOCK

FUNCTION_BLOCK AO
(* Analoge Ausgabe (AO): Hand/Automatik, Begrenzung, Rampe, Skalierung auf den Ausgabebereich
   ManualMode: PV follows Manual, else Auto; limited to Min..Max and to Ramp [unit/s] (0 = none);
   Out is PV scaled from EngLo..EngHi to OutLo..OutHi (e.g. 0..100 % -> 0..10 V). *)
VAR_INPUT
    Auto : REAL; ManualMode : BOOL; Manual : REAL;
    Min : REAL := 0.0; Max : REAL := 100.0; Ramp : REAL;
    EngLo : REAL := 0.0; EngHi : REAL := 100.0; OutLo : REAL := 0.0; OutHi : REAL := 10.0;
END_VAR
VAR_OUTPUT PV : REAL; Out : REAL; END_VAR
VAR T : __Dt; Target : REAL; Delta : REAL; Init : BOOL; END_VAR
T();
IF ManualMode THEN Target := Manual; ELSE Target := Auto; END_IF;
Target := LIMIT(Min, Target, Max);
IF NOT Init OR Ramp <= 0.0 THEN
    PV := Target; Init := TRUE;
ELSE
    Delta := Ramp * T.Dt;
    PV := LIMIT(PV - Delta, Target, PV + Delta);
END_IF;
IF EngHi <> EngLo THEN Out := OutLo + (PV - EngLo) * (OutHi - OutLo) / (EngHi - EngLo); ELSE Out := OutLo; END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK BO
(* Binäre Ausgabe (BO): Hand/Automatik, Mindestein- und Mindestausschaltzeit, Schaltspiele
   ManualMode: Manual, else Auto; a change waits until PV was on for MinOn / off for MinOff (also in
   manual mode: they protect the equipment). Q = PV XOR Invert. *)
VAR_INPUT Auto : BOOL; ManualMode : BOOL; Manual : BOOL; MinOn : TIME; MinOff : TIME; Invert : BOOL; END_VAR
VAR_OUTPUT Q : BOOL; PV : BOOL; Switches : UDINT; END_VAR
VAR T : __Dt; Want : BOOL; Since : TIME := T#24d; END_VAR
T();
IF ManualMode THEN Want := Manual; ELSE Want := Auto; END_IF;
IF Since < T#24d THEN Since := Since + T.DtT; END_IF;
IF Want <> PV AND ((PV AND Since >= MinOn) OR (NOT PV AND Since >= MinOff)) THEN
    PV := Want; Since := T#0s; Switches := Switches + 1;
END_IF;
Q := PV XOR Invert;
END_FUNCTION_BLOCK
`,
	},
	{
		group: "VDI 3814 · 1.2 E/A Werte",
		source: `
FUNCTION_BLOCK AV
(* Analogwert (AV): bedienbarer Wert (Sollwert, Parameter) mit Grenzen und Rückfallwert
   Starts with Default; a rising edge of Set takes In, of Reset Default. Value may also be written
   directly (HMI); it is kept within Min..Max. *)
VAR_INPUT In : REAL; Set : BOOL; Reset : BOOL; Default : REAL; Min : REAL := -1.0E30; Max : REAL := 1.0E30; END_VAR
VAR_OUTPUT Value : REAL; Changed : BOOL; END_VAR
VAR S : R_TRIG; R : R_TRIG; Init : BOOL; Old : REAL; END_VAR
IF NOT Init THEN Value := Default; Old := Default; Init := TRUE; END_IF;
S(CLK := Set);
R(CLK := Reset);
IF R.Q THEN Value := Default; ELSIF S.Q THEN Value := In; END_IF;
Value := LIMIT(Min, Value, Max);
Changed := Value <> Old;
Old := Value;
END_FUNCTION_BLOCK

FUNCTION_BLOCK BV
(* Binärwert (BV): bedienbarer Schaltwert - Setzen, Rücksetzen, Umschalten (Flanken)
   Starts with Default; Reset wins over Set over Toggle. Value may also be written directly (HMI). *)
VAR_INPUT Set : BOOL; Reset : BOOL; Toggle : BOOL; Default : BOOL; END_VAR
VAR_OUTPUT Value : BOOL; END_VAR
VAR S : R_TRIG; R : R_TRIG; Tg : R_TRIG; Init : BOOL; END_VAR
IF NOT Init THEN Value := Default; Init := TRUE; END_IF;
S(CLK := Set);
R(CLK := Reset);
Tg(CLK := Toggle);
IF R.Q THEN Value := FALSE; ELSIF S.Q THEN Value := TRUE; ELSIF Tg.Q THEN Value := NOT Value; END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK MV
(* Mehrstufenwert (MV): Stufe 1..States, z. B. Betriebsart Aus/Stufe 1/Stufe 2/Automatik
   Starts with Default; Set (edge) takes In, Up/Down (edges) step; kept within 1..States. Q1..Q4: the
   value is 1..4. Value may also be written directly (HMI). *)
VAR_INPUT States : INT := 3; In : INT; Set : BOOL; Up : BOOL; Down : BOOL; Default : INT := 1; END_VAR
VAR_OUTPUT Value : INT; Q1 : BOOL; Q2 : BOOL; Q3 : BOOL; Q4 : BOOL; END_VAR
VAR S : R_TRIG; U : R_TRIG; Dn : R_TRIG; Init : BOOL; END_VAR
IF NOT Init THEN Value := Default; Init := TRUE; END_IF;
S(CLK := Set);
U(CLK := Up);
Dn(CLK := Down);
IF S.Q THEN Value := In; ELSIF U.Q THEN Value := Value + 1; ELSIF Dn.Q THEN Value := Value - 1; END_IF;
Value := LIMIT(1, Value, MAX(States, 1));
Q1 := Value = 1; Q2 := Value = 2; Q3 := Value = 3; Q4 := Value = 4;
END_FUNCTION_BLOCK
`,
	},
	{
		group: "VDI 3814 · 1.3 E/A komplex",
		source: `
FUNCTION_BLOCK CLOCK
(* Uhr: Datum, Uhrzeit, Wochentag aus dem SPS-Takt, stellbar (für Zeitplan und Kalender)
   The PLC has no wall clock yet: a rising edge of Set (or the first call) takes Year..Second, then the
   clock runs with the PLC time (Speed: 60.0 = one minute per second, for tests). Weekday 1 = Monday ..
   7 = Sunday. *)
VAR_INPUT Set : BOOL; Year : INT := 2026; Month : INT := 1; Day : INT := 1; Hour : INT; Minute : INT; Second : INT; Speed : REAL := 1.0; END_VAR
VAR_OUTPUT TimeOfDay : TIME; Weekday : INT; DayOfYear : INT; Y : INT; Mo : INT; D : INT; H : INT; Mi : INT; S : INT; END_VAR
VAR T : __Dt; Edge : R_TRIG; Init : BOOL; Days : DINT; Ms : LREAL; Msi : DINT;
    Z : DINT; Era : DINT; Doe : DINT; Yoe : DINT; Doy : DINT; Mp : DINT; Yy : DINT; END_VAR
T();
Edge(CLK := Set);
IF Edge.Q OR NOT Init THEN
    Days := __Days(Year, Month, Day);
    Ms := INT_TO_LREAL(Hour) * 3600000.0 + INT_TO_LREAL(Minute) * 60000.0 + INT_TO_LREAL(Second) * 1000.0;
    Init := TRUE;
ELSE
    Ms := Ms + T.Dt * 1000.0 * MAX(Speed, 0.0);
END_IF;
WHILE Ms >= 86400000.0 DO
    Ms := Ms - 86400000.0; Days := Days + 1;
END_WHILE;
IF Ms < 0.0 THEN Ms := 0.0; END_IF;
Msi := TRUNC(Ms);
TimeOfDay := DINT_TO_TIME(Msi);
H := DINT_TO_INT(Msi / 3600000);
Mi := DINT_TO_INT((Msi / 60000) MOD 60);
S := DINT_TO_INT((Msi / 1000) MOD 60);
Weekday := DINT_TO_INT((Days + 3) MOD 7) + 1;
Z := Days + 719468;
Era := Z / 146097;
Doe := Z - Era * 146097;
Yoe := (Doe - Doe / 1460 + Doe / 36524 - Doe / 146096) / 365;
Yy := Yoe + Era * 400;
Doy := Doe - (365 * Yoe + Yoe / 4 - Yoe / 100);
Mp := (5 * Doy + 2) / 153;
D := DINT_TO_INT(Doy - (153 * Mp + 2) / 5 + 1);
IF Mp < 10 THEN Mo := DINT_TO_INT(Mp + 3); ELSE Mo := DINT_TO_INT(Mp - 9); END_IF;
IF Mo <= 2 THEN Yy := Yy + 1; END_IF;
Y := DINT_TO_INT(Yy);
DayOfYear := DINT_TO_INT(Days - __Days(Y, 1, 1) + 1);
END_FUNCTION_BLOCK

FUNCTION_BLOCK SCH
(* Zeitplan (SCH): Wochenschaltprogramm Mo-Fr (zwei Zeiten), Sa, So; Feiertag wie Sonntag
   Q is TRUE inside a period [On, Off) of the day (Off before On: over midnight, On = Off: none);
   Value is OnValue or OffValue (e.g. a setpoint). Weekday 1..7 and TimeOfDay from VDI3814.CLOCK. *)
VAR_INPUT
    Weekday : INT := 1; TimeOfDay : TIME; Holiday : BOOL; Enable : BOOL := TRUE;
    WorkOn1 : TIME := T#7h; WorkOff1 : TIME := T#18h; WorkOn2 : TIME; WorkOff2 : TIME;
    SatOn : TIME; SatOff : TIME; SunOn : TIME; SunOff : TIME;
    OnValue : REAL := 1.0; OffValue : REAL := 0.0;
END_VAR
VAR_OUTPUT Q : BOOL; Value : REAL; END_VAR
IF NOT Enable THEN
    Q := FALSE;
ELSIF Holiday OR Weekday = 7 THEN
    Q := __Within(TimeOfDay, SunOn, SunOff);
ELSIF Weekday = 6 THEN
    Q := __Within(TimeOfDay, SatOn, SatOff);
ELSE
    Q := __Within(TimeOfDay, WorkOn1, WorkOff1) OR __Within(TimeOfDay, WorkOn2, WorkOff2);
END_IF;
IF Q THEN Value := OnValue; ELSE Value := OffValue; END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK CAL
(* Kalender (CAL): bundesweite Feiertage, bis zu 8 Sondertage und ein Zeitraum (Betriebsferien)
   Dates as MMDD (1225 = 25 December, 0 = unused); the range RangeFrom..RangeTo may cross the year
   (1224..0106). Holiday: a nationwide German holiday (New Year, Good Friday, Easter Monday, 1 May,
   Ascension, Whit Monday, 3 October, Christmas) when GermanHolidays. Q: any of them today. *)
VAR_INPUT
    Year : INT := 2026; Month : INT := 1; Day : INT := 1; GermanHolidays : BOOL := TRUE;
    D1 : INT; D2 : INT; D3 : INT; D4 : INT; D5 : INT; D6 : INT; D7 : INT; D8 : INT;
    RangeFrom : INT; RangeTo : INT;
END_VAR
VAR_OUTPUT Q : BOOL; Holiday : BOOL; Special : BOOL; END_VAR
VAR CachedYear : INT; Easter : DINT; Today : DINT; Md : INT;
    A : INT; B : INT; C : INT; Dd : INT; E : INT; F : INT; G : INT; H : INT; I : INT; K : INT; L : INT; M : INT; Em : INT; Ed : INT; END_VAR
IF Year <> CachedYear THEN
    (* Easter Sunday, anonymous Gregorian algorithm *)
    A := Year MOD 19; B := Year / 100; C := Year MOD 100; Dd := B / 4; E := B MOD 4;
    F := (B + 8) / 25; G := (B - F + 1) / 3; H := (19 * A + B - Dd - G + 15) MOD 30;
    I := C / 4; K := C MOD 4; L := (32 + 2 * E + 2 * I - H - K) MOD 7; M := (A + 11 * H + 22 * L) / 451;
    Em := (H + L - 7 * M + 114) / 31; Ed := ((H + L - 7 * M + 114) MOD 31) + 1;
    Easter := __Days(Year, Em, Ed);
    CachedYear := Year;
END_IF;
Md := Month * 100 + Day;
Today := __Days(Year, Month, Day);
Holiday := GermanHolidays AND (Md = 101 OR Md = 501 OR Md = 1003 OR Md = 1225 OR Md = 1226 OR
    Today = Easter - 2 OR Today = Easter + 1 OR Today = Easter + 39 OR Today = Easter + 50);
Special := (Md = D1 OR Md = D2 OR Md = D3 OR Md = D4 OR Md = D5 OR Md = D6 OR Md = D7 OR Md = D8);
IF RangeFrom > 0 AND RangeTo > 0 THEN
    IF RangeFrom <= RangeTo THEN
        Special := Special OR (Md >= RangeFrom AND Md <= RangeTo);
    ELSE
        Special := Special OR Md >= RangeFrom OR Md <= RangeTo;
    END_IF;
END_IF;
Q := Holiday OR Special;
END_FUNCTION_BLOCK

FUNCTION_BLOCK NC
(* Alarm-/Ereignismeldung (NC): Meldung mit Verzögerung, Quittierung, Zustand und Meldeleuchte
   Event must last Delay to count. State: 0 normal, 1 active not acknowledged, 2 active acknowledged,
   3 gone not acknowledged. Ack (rising edge) acknowledges; NeedAck = FALSE: no acknowledgement needed.
   Lamp blinks (1 Hz) while unacknowledged, is steady while active. Prio: the message class
   (1 danger to life, 2 safety, 3 alarm, 4 fault, 5 maintenance, 6 system, 7 manual intervention, 8 other). *)
VAR_INPUT Event : BOOL; Ack : BOOL; Delay : TIME; Prio : INT := 3; NeedAck : BOOL := TRUE; END_VAR
VAR_OUTPUT Active : BOOL; Unacked : BOOL; Alarm : BOOL; Lamp : BOOL; State : INT; Count : UDINT; END_VAR
VAR Dly : TON; AckEdge : R_TRIG; Last : BOOL; END_VAR
Dly(IN := Event, PT := Delay);
Active := Dly.Q;
AckEdge(CLK := Ack);
IF Active AND NOT Last THEN
    Count := Count + 1;
    Unacked := NeedAck;
END_IF;
IF AckEdge.Q OR NOT NeedAck THEN Unacked := FALSE; END_IF;
Last := Active;
Alarm := Active OR Unacked;
IF Active AND Unacked THEN State := 1; ELSIF Active THEN State := 2; ELSIF Unacked THEN State := 3; ELSE State := 0; END_IF;
IF Unacked THEN
    Lamp := (TIME_TO_DINT(__PLC_TIME()) / 500) MOD 2 = 0;
ELSE
    Lamp := Active;
END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK LOG
(* Datenaufzeichnung (LOG): Ringspeicher mit 60 Werten im Intervall, Minimum, Maximum, Mittelwert
   Every Interval (while Enable) In is stored; Buf[Next] is the next place, Count the values stored
   (at most 60). Min, Max, Avg over the stored values. Clear (TRUE) empties it. *)
VAR_INPUT In : REAL; Interval : TIME := T#1m; Enable : BOOL := TRUE; Clear : BOOL; END_VAR
VAR_OUTPUT Count : INT; Last : REAL; Min : REAL; Max : REAL; Avg : REAL; END_VAR
VAR T : __Dt; Buf : ARRAY[0..59] OF REAL; Next : INT; Elapsed : TIME; I : INT; Sum : REAL; Started : BOOL; END_VAR
T();
IF Clear THEN
    Count := 0; Next := 0; Elapsed := T#0s; Started := FALSE;
    Min := 0.0; Max := 0.0; Avg := 0.0; Last := 0.0;
ELSIF Enable THEN
    Elapsed := Elapsed + T.DtT;
    IF NOT Started OR Elapsed >= Interval THEN
        Started := TRUE;
        Elapsed := T#0s;
        Buf[Next] := In; Last := In;
        Next := (Next + 1) MOD 60;
        IF Count < 60 THEN Count := Count + 1; END_IF;
        Min := Buf[0]; Max := Buf[0]; Sum := 0.0;
        FOR I := 0 TO Count - 1 DO
            Min := MIN(Min, Buf[I]); Max := MAX(Max, Buf[I]); Sum := Sum + Buf[I];
        END_FOR;
        Avg := Sum / INT_TO_REAL(Count);
    END_IF;
END_IF;
END_FUNCTION_BLOCK
`,
	},
	{
		group: "VDI 3814 · 2.1 Logik",
		source: `
FUNCTION SCALE : REAL
(* Arithmetische Berechnung: lineare Umrechnung von InLo..InHi auf OutLo..OutHi *)
VAR_INPUT In : REAL; InLo : REAL; InHi : REAL; OutLo : REAL; OutHi : REAL; END_VAR
IF InHi <> InLo THEN SCALE := OutLo + (In - InLo) * (OutHi - OutLo) / (InHi - InLo); ELSE SCALE := OutLo; END_IF;
END_FUNCTION

FUNCTION_BLOCK MEAN4
(* Arithmetische Berechnung: Mittelwert, Minimum, Maximum über N von 4 Eingängen (z. B. Raumfühler) *)
VAR_INPUT In1 : REAL; In2 : REAL; In3 : REAL; In4 : REAL; N : INT := 2; END_VAR
VAR_OUTPUT Avg : REAL; Min : REAL; Max : REAL; END_VAR
VAR V : ARRAY[1..4] OF REAL; I : INT; Cnt : INT; Sum : REAL; END_VAR
V[1] := In1; V[2] := In2; V[3] := In3; V[4] := In4;
Cnt := LIMIT(1, N, 4);
Min := V[1]; Max := V[1]; Sum := 0.0;
FOR I := 1 TO Cnt DO
    Min := MIN(Min, V[I]); Max := MAX(Max, V[I]); Sum := Sum + V[I];
END_FOR;
Avg := Sum / INT_TO_REAL(Cnt);
END_FUNCTION_BLOCK

FUNCTION_BLOCK TONOF
(* Zeiten: Ein- und Ausschaltverzögerung in einem Baustein *)
VAR_INPUT IN : BOOL; TOn : TIME; TOff : TIME; END_VAR
VAR_OUTPUT Q : BOOL; END_VAR
VAR A : TON; B : TON; END_VAR
A(IN := IN AND NOT Q, PT := TOn);
B(IN := NOT IN AND Q, PT := TOff);
IF A.Q THEN Q := TRUE; ELSIF B.Q THEN Q := FALSE; END_IF;
IF IN AND TOn = T#0s THEN Q := TRUE; END_IF;
IF NOT IN AND TOff = T#0s THEN Q := FALSE; END_IF;
END_FUNCTION_BLOCK
`,
	},
	{
		group: "VDI 3814 · 2.2 Überwachen/Steuern",
		source: `
FUNCTION_BLOCK LIMITMON
(* Grenzwertüberwachung: oberer und unterer Grenzwert mit Schaltdifferenz und Verzögerung
   QHigh when In > High (released below High - Hyst), QLow when In < Low (released above Low + Hyst),
   each after it lasted Delay. *)
VAR_INPUT In : REAL; High : REAL := 1.0E30; Low : REAL := -1.0E30; Hyst : REAL; Delay : TIME; Enable : BOOL := TRUE; END_VAR
VAR_OUTPUT QHigh : BOOL; QLow : BOOL; Q : BOOL; END_VAR
VAR H : BOOL; L : BOOL; TH : TON; TL : TON; END_VAR
IF NOT Enable THEN
    H := FALSE; L := FALSE;
ELSE
    IF In > High THEN H := TRUE; ELSIF In < High - Hyst THEN H := FALSE; END_IF;
    IF In < Low THEN L := TRUE; ELSIF In > Low + Hyst THEN L := FALSE; END_IF;
END_IF;
TH(IN := H, PT := Delay);
TL(IN := L, PT := Delay);
QHigh := TH.Q; QLow := TL.Q; Q := QHigh OR QLow;
END_FUNCTION_BLOCK

FUNCTION_BLOCK RUNHOURS
(* Betriebsstundenüberwachung: Betriebsstunden, Starts, Wartungsintervall
   Hours counts while Run; Starts counts its rising edges. SinceService counts from the last
   ServiceDone (edge); ServiceDue when it reaches ServiceInterval [h] (0 = none). SetPreset (edge) sets
   Hours to Preset (meter exchange). *)
VAR_INPUT Run : BOOL; ServiceInterval : REAL; ServiceDone : BOOL; Preset : LREAL; SetPreset : BOOL; END_VAR
VAR_OUTPUT Hours : LREAL; Starts : UDINT; SinceService : LREAL; ServiceDue : BOOL; END_VAR
VAR T : __Dt; Last : BOOL; Sd : R_TRIG; Sp : R_TRIG; END_VAR
T();
Sd(CLK := ServiceDone);
Sp(CLK := SetPreset);
IF Sp.Q THEN Hours := Preset; END_IF;
IF Sd.Q THEN SinceService := 0.0; END_IF;
IF Run THEN
    Hours := Hours + T.Dt / 3600.0;
    SinceService := SinceService + T.Dt / 3600.0;
    IF NOT Last THEN Starts := Starts + 1; END_IF;
END_IF;
Last := Run;
ServiceDue := ServiceInterval > 0.0 AND SinceService >= ServiceInterval;
END_FUNCTION_BLOCK

FUNCTION_BLOCK EVENTCOUNT
(* Ereignisüberwachung (Zählung): Ereignisse zählen, Grenze, Häufigkeit je Zeitfenster
   Count counts rising edges of In (Reset clears); LimitReached when Count >= Limit (0 = none).
   InWindow: the events of the current Window, Frequent when more than MaxPerWindow (0 = none),
   e.g. burner starts per hour. *)
VAR_INPUT In : BOOL; Reset : BOOL; Limit : UDINT; Window : TIME := T#1h; MaxPerWindow : UDINT; END_VAR
VAR_OUTPUT Count : UDINT; LimitReached : BOOL; InWindow : UDINT; Frequent : BOOL; END_VAR
VAR T : __Dt; Edge : R_TRIG; Elapsed : TIME; END_VAR
T();
Edge(CLK := In);
IF Reset THEN
    Count := 0; InWindow := 0; Elapsed := T#0s; Frequent := FALSE;
ELSE
    Elapsed := Elapsed + T.DtT;
    IF Elapsed >= Window THEN Elapsed := T#0s; InWindow := 0; END_IF;
    IF Edge.Q THEN Count := Count + 1; InWindow := InWindow + 1; END_IF;
    Frequent := MaxPerWindow > 0 AND InWindow > MaxPerWindow;
END_IF;
LimitReached := Limit > 0 AND Count >= Limit;
END_FUNCTION_BLOCK

FUNCTION_BLOCK CMDMON
(* Befehlsausführüberwachung: Befehl gegen Rückmeldung, Störung verriegelt bis Reset
   NoFeedback: Cmd without Feedback for OnDelay; Unexpected: Feedback without Cmd for OffDelay. Both
   latch until a rising edge of Reset. *)
VAR_INPUT Cmd : BOOL; Feedback : BOOL; OnDelay : TIME := T#10s; OffDelay : TIME := T#10s; Reset : BOOL; END_VAR
VAR_OUTPUT Fault : BOOL; NoFeedback : BOOL; Unexpected : BOOL; END_VAR
VAR A : TON; B : TON; R : R_TRIG; END_VAR
A(IN := Cmd AND NOT Feedback, PT := OnDelay);
B(IN := NOT Cmd AND Feedback, PT := OffDelay);
R(CLK := Reset);
IF R.Q THEN NoFeedback := FALSE; Unexpected := FALSE; END_IF;
IF A.Q THEN NoFeedback := TRUE; END_IF;
IF B.Q THEN Unexpected := TRUE; END_IF;
Fault := NoFeedback OR Unexpected;
END_FUNCTION_BLOCK

FUNCTION_BLOCK DEVSTATUS
(* Anlagen-/Gerätestatus: Zustand aus Befehl und Rückmeldungen
   Status: 0 off, 1 on, 2 starting/stopping (command and feedback differ), 3 local manual,
   4 maintenance, 5 fault (highest wins). *)
VAR_INPUT Cmd : BOOL; Running : BOOL; Fault : BOOL; Maintenance : BOOL; LocalManual : BOOL; END_VAR
VAR_OUTPUT Status : INT; IsOff : BOOL; IsOn : BOOL; Changing : BOOL; Manual : BOOL; Service : BOOL; Faulted : BOOL; END_VAR
IF Fault THEN Status := 5;
ELSIF Maintenance THEN Status := 4;
ELSIF LocalManual THEN Status := 3;
ELSIF Cmd <> Running THEN Status := 2;
ELSIF Running THEN Status := 1;
ELSE Status := 0;
END_IF;
IsOff := Status = 0; IsOn := Status = 1; Changing := Status = 2; Manual := Status = 3; Service := Status = 4; Faulted := Status = 5;
END_FUNCTION_BLOCK

FUNCTION_BLOCK MOTOR
(* Motorsteuerung: Hand-0-Automatik, Freigabe, Betriebsrückmeldung, Motorschutz, Störung mit Quittierung
   Mode 0 off, 1 manual on, 2 automatic (Auto). Release FALSE stops it in every mode (interlock). A fault
   latches and stops the motor until Reset (rising edge) with its cause gone: FaultCode 1 motor
   protection (Overload), 2 no run feedback within FbDelay, 3 feedback without command (only when
   UseFeedback). MinOff: restart lock after stopping. Status: 0 off, 1 running, 2 fault, 3 starting. *)
VAR_INPUT Auto : BOOL; Mode : INT := 2; Release : BOOL := TRUE; Feedback : BOOL; Overload : BOOL; FbDelay : TIME := T#5s;
    UseFeedback : BOOL := TRUE; Reset : BOOL; MinOff : TIME; END_VAR
VAR_OUTPUT Q : BOOL; Running : BOOL; Fault : BOOL; FaultCode : INT; Status : INT; END_VAR
VAR T : __Dt; Want : BOOL; NoFb : TON; Unexp : TON; R : R_TRIG; OffFor : TIME := T#24d; END_VAR
T();
R(CLK := Reset);
IF R.Q AND NOT Overload THEN Fault := FALSE; FaultCode := 0; END_IF;
IF Overload THEN Fault := TRUE; FaultCode := 1; END_IF;
Want := NOT Fault AND Release AND (Mode = 1 OR (Mode = 2 AND Auto));
IF Q THEN
    OffFor := T#0s;
ELSIF OffFor < T#24d THEN
    OffFor := OffFor + T.DtT;
END_IF;
IF Want AND NOT Q AND OffFor >= MinOff THEN Q := TRUE; ELSIF NOT Want THEN Q := FALSE; END_IF;
NoFb(IN := UseFeedback AND Q AND NOT Feedback, PT := FbDelay);
Unexp(IN := UseFeedback AND NOT Q AND Feedback, PT := FbDelay);
IF NoFb.Q AND NOT Fault THEN Fault := TRUE; FaultCode := 2; Q := FALSE; END_IF;
IF Unexp.Q AND NOT Fault THEN Fault := TRUE; FaultCode := 3; END_IF;
IF Fault THEN Q := FALSE; END_IF;
Running := Q AND (Feedback OR NOT UseFeedback);
IF Fault THEN Status := 2; ELSIF Running THEN Status := 1; ELSIF Q THEN Status := 3; ELSE Status := 0; END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK ANTISEIZE
(* Blockierschutz: Pumpe/Ventil nach Stillstandszeit kurz bewegen
   Q = Cmd, and after Interval without running (Enable, not Inhibit) once for Duration. *)
VAR_INPUT Cmd : BOOL; Enable : BOOL := TRUE; Interval : TIME := T#24h; Duration : TIME := T#30s; Inhibit : BOOL; END_VAR
VAR_OUTPUT Q : BOOL; Exercising : BOOL; END_VAR
VAR T : __Dt; Idle : TIME; Run : TIME; END_VAR
T();
IF Exercising THEN
    Run := Run + T.DtT;
    IF Run >= Duration OR Inhibit OR NOT Enable THEN Exercising := FALSE; Idle := T#0s; END_IF;
ELSIF Cmd THEN
    Idle := T#0s;
ELSE
    IF Idle < Interval THEN Idle := Idle + T.DtT; END_IF;
    IF Idle >= Interval AND Enable AND NOT Inhibit THEN Exercising := TRUE; Run := T#0s; END_IF;
END_IF;
Q := Cmd OR Exercising;
END_FUNCTION_BLOCK

FUNCTION_BLOCK ASWITCH
(* Umschaltung (Analogwert): einen von vier Werten wählen, stoßfrei über eine Rampe
   Sel 0..3 chooses In0..In3; Out follows with at most Ramp [unit/s] (0 = at once). *)
VAR_INPUT Sel : INT; In0 : REAL; In1 : REAL; In2 : REAL; In3 : REAL; Ramp : REAL; END_VAR
VAR_OUTPUT Out : REAL; END_VAR
VAR T : __Dt; Target : REAL; Init : BOOL; Delta : REAL; END_VAR
T();
CASE Sel OF
    1: Target := In1;
    2: Target := In2;
    3: Target := In3;
ELSE
    Target := In0;
END_CASE;
IF NOT Init OR Ramp <= 0.0 THEN
    Out := Target; Init := TRUE;
ELSE
    Delta := Ramp * T.Dt;
    Out := LIMIT(Out - Delta, Target, Out + Delta);
END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SEQUENCE
(* Folgesteuerung: bis zu 4 Stufen/Aggregate nach Bedarf zu- und abschalten, Führungswechsel
   Demand 0..100 %: stage k (of Stages) is added above (k-1)*100/Stages + Hyst for OnDelay and removed
   at or below (k-1)*100/Stages for OffDelay. The stages run in the order Lead, Lead+1, ...; unavailable
   ones (AvailX FALSE) are skipped. A rising edge of Rotate makes the next one lead. *)
VAR_INPUT Demand : REAL; Stages : INT := 4; OnDelay : TIME := T#2m; OffDelay : TIME := T#2m; Hyst : REAL := 5.0; Rotate : BOOL;
    Avail1 : BOOL := TRUE; Avail2 : BOOL := TRUE; Avail3 : BOOL := TRUE; Avail4 : BOOL := TRUE; END_VAR
VAR_OUTPUT Q1 : BOOL; Q2 : BOOL; Q3 : BOOL; Q4 : BOOL; Active : INT; Lead : INT := 1; END_VAR
VAR Up : TON; Down : TON; Rot : R_TRIG; N : INT; StepPct : REAL; Av : ARRAY[1..4] OF BOOL; Qa : ARRAY[1..4] OF BOOL;
    I : INT; K : INT; Left : INT; END_VAR
N := LIMIT(1, Stages, 4);
StepPct := 100.0 / INT_TO_REAL(N);
Rot(CLK := Rotate);
IF Rot.Q THEN Lead := Lead MOD N + 1; END_IF;
Lead := LIMIT(1, Lead, N);
Up(IN := Active < N AND Demand > INT_TO_REAL(Active) * StepPct + Hyst, PT := OnDelay);
Down(IN := Active > 0 AND Demand <= INT_TO_REAL(Active - 1) * StepPct, PT := OffDelay);
IF Up.Q THEN Active := Active + 1; Up(IN := FALSE, PT := OnDelay);
ELSIF Down.Q THEN Active := Active - 1; Down(IN := FALSE, PT := OffDelay);
END_IF;
Active := LIMIT(0, Active, N);
Av[1] := Avail1; Av[2] := Avail2; Av[3] := Avail3; Av[4] := Avail4;
Left := Active;
FOR I := 1 TO 4 DO
    Qa[I] := FALSE;
END_FOR;
FOR I := 0 TO N - 1 DO
    K := (Lead - 1 + I) MOD N + 1;
    IF Left > 0 AND Av[K] THEN Qa[K] := TRUE; Left := Left - 1; END_IF;
END_FOR;
Q1 := Qa[1]; Q2 := Qa[2]; Q3 := Qa[3]; Q4 := Qa[4];
END_FUNCTION_BLOCK

FUNCTION_BLOCK FROST
(* Frostschutzsteuerung: Vorwarnung öffnet das Heizventil, Auslösung schaltet ab (verriegelt)
   Y opens the heating valve proportionally from PreLimit (0 %) to TripLimit (100 %) - an override for
   the controller output (use MAX). Trip when Temp < TripLimit for Delay or the frost thermostat
   (Thermostat TRUE = frost): Y 100 %, Release FALSE (fan off, outside air damper closed) until Reset with
   Temp above TripLimit + Hyst. *)
VAR_INPUT Temp : REAL; PreLimit : REAL := 8.0; TripLimit : REAL := 5.0; Hyst : REAL := 2.0; Thermostat : BOOL; Delay : TIME := T#2s; Reset : BOOL; END_VAR
VAR_OUTPUT Y : REAL; PreAlarm : BOOL; Trip : BOOL; Release : BOOL; END_VAR
VAR D : TON; R : R_TRIG; END_VAR
D(IN := Temp < TripLimit OR Thermostat, PT := Delay);
R(CLK := Reset);
IF D.Q THEN Trip := TRUE; END_IF;
IF R.Q AND NOT Thermostat AND Temp > TripLimit + Hyst THEN Trip := FALSE; END_IF;
PreAlarm := Temp < PreLimit;
IF Trip THEN
    Y := 100.0;
ELSIF PreLimit > TripLimit THEN
    Y := LIMIT(0.0, (PreLimit - Temp) / (PreLimit - TripLimit) * 100.0, 100.0);
ELSE
    Y := 0.0;
END_IF;
Release := NOT Trip;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SAFETY
(* Sicherheitssteuerung: Sicherheitskette aus bis zu 4 Kontakten, verriegelt, Erstwertmeldung
   OKx TRUE = safe (closed contact). One open contact trips; FirstOut is the first one (1..4). Reset
   (rising edge) only with every contact closed. Release = no trip. *)
VAR_INPUT OK1 : BOOL := TRUE; OK2 : BOOL := TRUE; OK3 : BOOL := TRUE; OK4 : BOOL := TRUE; Reset : BOOL; END_VAR
VAR_OUTPUT Release : BOOL; Trip : BOOL; FirstOut : INT; END_VAR
VAR R : R_TRIG; AllOk : BOOL; END_VAR
R(CLK := Reset);
AllOk := OK1 AND OK2 AND OK3 AND OK4;
IF NOT Trip AND NOT AllOk THEN
    Trip := TRUE;
    IF NOT OK1 THEN FirstOut := 1; ELSIF NOT OK2 THEN FirstOut := 2; ELSIF NOT OK3 THEN FirstOut := 3; ELSE FirstOut := 4; END_IF;
ELSIF Trip AND R.Q AND AllOk THEN
    Trip := FALSE; FirstOut := 0;
END_IF;
Release := NOT Trip;
END_FUNCTION_BLOCK

FUNCTION_BLOCK PRIORITY
(* Prioritätssteuerung: 8 Prioritätsebenen (1 = höchste) mit Freigabe je Ebene, sonst Rückfallwert
   The enabled level with the smallest number wins (e.g. 1 safety, 2 frost, 5 manual, 8 automatic);
   none: Default. Q: Out <> 0. *)
VAR_INPUT
    En1 : BOOL; V1 : REAL; En2 : BOOL; V2 : REAL; En3 : BOOL; V3 : REAL; En4 : BOOL; V4 : REAL;
    En5 : BOOL; V5 : REAL; En6 : BOOL; V6 : REAL; En7 : BOOL; V7 : REAL; En8 : BOOL; V8 : REAL;
    Default : REAL;
END_VAR
VAR_OUTPUT Out : REAL; Q : BOOL; Level : INT; END_VAR
IF En1 THEN Out := V1; Level := 1;
ELSIF En2 THEN Out := V2; Level := 2;
ELSIF En3 THEN Out := V3; Level := 3;
ELSIF En4 THEN Out := V4; Level := 4;
ELSIF En5 THEN Out := V5; Level := 5;
ELSIF En6 THEN Out := V6; Level := 6;
ELSIF En7 THEN Out := V7; Level := 7;
ELSIF En8 THEN Out := V8; Level := 8;
ELSE Out := Default; Level := 0;
END_IF;
Q := Out <> 0.0;
END_FUNCTION_BLOCK
`,
	},
	{
		group: "VDI 3814 · 2.3 Regelung",
		source: `
FUNCTION_BLOCK PID
(* P-/PI-/PID-Regler: Stellungsalgorithmus mit Anti-Windup, Wirkrichtung, Hand, stoßfreier Umschaltung
   E = SP - PV (Reverse: PV - SP, e.g. cooling); |E| below Deadband counts as 0. Y = Kp*E + I + D + Offset
   within YMin..YMax; Tn [s] integral time (0: P only, Offset is the operating point), Tv [s] derivative
   time (filtered with Tv/8). The integral is held at the limits (anti-windup). Not Enable: Y = YOff;
   Manual: Y = YManual; both track the integral so that the change back is bumpless. *)
VAR_INPUT SP : REAL; PV : REAL; Kp : REAL := 1.0; Tn : REAL; Tv : REAL; Reverse : BOOL;
    YMin : REAL := 0.0; YMax : REAL := 100.0; Deadband : REAL; Enable : BOOL := TRUE;
    Manual : BOOL; YManual : REAL; YOff : REAL; Offset : REAL; END_VAR
VAR_OUTPUT Y : REAL; E : REAL; AtLimit : BOOL; END_VAR
VAR T : __Dt; I : REAL; D : REAL; P : REAL; EOld : REAL; Init : BOOL; Raw : REAL; END_VAR
T();
E := SP - PV;
IF Reverse THEN E := -E; END_IF;
IF ABS(E) < Deadband THEN E := 0.0; END_IF;
P := Kp * E;
IF NOT Enable OR Manual THEN
    IF Manual AND Enable THEN Y := LIMIT(YMin, YManual, YMax); ELSE Y := LIMIT(YMin, YOff, YMax); END_IF;
    IF Tn > 0.0 THEN I := Y - P - Offset; ELSE I := 0.0; END_IF;
    D := 0.0;
ELSE
    IF Tv > 0.0 AND Init AND T.Dt > 0.0 THEN
        Raw := Kp * Tv * (E - EOld) / T.Dt;
        D := D + (Raw - D) * T.Dt / (T.Dt + Tv / 8.0);
    ELSE
        D := 0.0;
    END_IF;
    IF Tn > 0.0 THEN
        I := I + Kp * E * T.Dt / Tn;
        I := LIMIT(YMin - P - Offset, I, YMax - P - Offset);
    ELSE
        I := 0.0;
    END_IF;
    Y := LIMIT(YMin, P + I + D + Offset, YMax);
END_IF;
AtLimit := Y <= YMin OR Y >= YMax;
EOld := E;
Init := TRUE;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SPCURVE
(* Sollwertführung/-kennlinie: Kennlinie durch 4 Punkte (z. B. Heizkurve), Verschiebung, Begrenzung
   Y over X through (X1,Y1)..(X4,Y4) (X ascending), linear between the points, constant beyond the end
   points; Shift is added, then limited to YMin..YMax. Default: a heating curve outside temperature ->
   flow temperature. *)
VAR_INPUT X : REAL; X1 : REAL := -15.0; Y1 : REAL := 70.0; X2 : REAL := 0.0; Y2 : REAL := 52.0; X3 : REAL := 10.0; Y3 : REAL := 38.0;
    X4 : REAL := 20.0; Y4 : REAL := 22.0; Shift : REAL; YMin : REAL := -1.0E30; YMax : REAL := 1.0E30; END_VAR
VAR_OUTPUT Y : REAL; END_VAR
VAR Xs : ARRAY[1..4] OF REAL; Ys : ARRAY[1..4] OF REAL; I : INT; R : REAL; END_VAR
Xs[1] := X1; Xs[2] := X2; Xs[3] := X3; Xs[4] := X4;
Ys[1] := Y1; Ys[2] := Y2; Ys[3] := Y3; Ys[4] := Y4;
IF X <= Xs[1] THEN
    R := Ys[1];
ELSIF X >= Xs[4] THEN
    R := Ys[4];
ELSE
    R := Ys[4];
    FOR I := 1 TO 3 DO
        IF X >= Xs[I] AND X < Xs[I + 1] THEN
            R := Ys[I] + (X - Xs[I]) * (Ys[I + 1] - Ys[I]) / (Xs[I + 1] - Xs[I]);
            EXIT;
        END_IF;
    END_FOR;
END_IF;
Y := LIMIT(YMin, R + Shift, YMax);
END_FUNCTION_BLOCK

FUNCTION_BLOCK OUTCONT
(* Stellausgabe stetig: Stellgröße 0..100 % auf den Stellbereich des Antriebs (z. B. 2..10 V)
   Pos = Y within YMin..YMax (minimum/maximum position), at most Ramp [%/s] (0 = at once); Safe:
   YSafe. Out = OutLo..OutHi over 0..100 % (Invert: 100 % -> OutLo). *)
VAR_INPUT Y : REAL; YMin : REAL := 0.0; YMax : REAL := 100.0; OutLo : REAL := 0.0; OutHi : REAL := 10.0; Invert : BOOL;
    Safe : BOOL; YSafe : REAL; Ramp : REAL; END_VAR
VAR_OUTPUT Out : REAL; Pos : REAL; END_VAR
VAR T : __Dt; Target : REAL; Init : BOOL; Delta : REAL; END_VAR
T();
IF Safe THEN Target := LIMIT(0.0, YSafe, 100.0); ELSE Target := LIMIT(MAX(YMin, 0.0), Y, MIN(YMax, 100.0)); END_IF;
IF NOT Init OR Ramp <= 0.0 THEN
    Pos := Target; Init := TRUE;
ELSE
    Delta := Ramp * T.Dt;
    Pos := LIMIT(Pos - Delta, Target, Pos + Delta);
END_IF;
IF Invert THEN Out := OutHi - Pos / 100.0 * (OutHi - OutLo); ELSE Out := OutLo + Pos / 100.0 * (OutHi - OutLo); END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK OUT2P
(* Stellausgabe 2-Punkt: Stellgröße in Ein/Aus mit Schaltschwellen und Mindestzeiten
   Q on when Y >= OnAt, off when Y <= OffAt; a change waits for MinOn / MinOff. *)
VAR_INPUT Y : REAL; OnAt : REAL := 60.0; OffAt : REAL := 40.0; MinOn : TIME; MinOff : TIME; END_VAR
VAR_OUTPUT Q : BOOL; END_VAR
VAR T : __Dt; Since : TIME := T#24d; END_VAR
T();
IF Since < T#24d THEN Since := Since + T.DtT; END_IF;
IF NOT Q AND Y >= OnAt AND Since >= MinOff THEN
    Q := TRUE; Since := T#0s;
ELSIF Q AND Y <= OffAt AND Since >= MinOn THEN
    Q := FALSE; Since := T#0s;
END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK OUT3P
(* Stellausgabe 3-Punkt: Stellgröße auf Auf-/Zu-Impulse eines Antriebs mit Laufzeit
   Pos is the modelled actuator position (RunTime for 0..100 %). It moves when |Y - Pos| > Deadband
   (at least MinPulse); at Y >= 100 / Y <= 0 it drives SyncExtra % of the runtime further to synchronise
   at the end stop, then stays there (no new run) while Y stays at the end. The first call closes for
   the full runtime (+ 2 * SyncExtra) to know the position. *)
VAR_INPUT Y : REAL; RunTime : TIME := T#120s; Deadband : REAL := 2.0; MinPulse : TIME := T#200ms; SyncExtra : REAL := 10.0; END_VAR
VAR_OUTPUT Open : BOOL; Close : BOOL; Pos : REAL; Syncing : BOOL; END_VAR
VAR T : __Dt; Pi : REAL; Target : REAL; Moving : TIME; Init : BOOL; Rate : REAL;
    EndHold : INT; (* 1 / -1: synchronised at the open / closed end, Y still there: no new run *) END_VAR
T();
Rate := 100.0 / MAX(TIME_TO_REAL(RunTime) * 0.001, 0.1);
IF NOT Init THEN
    Pi := 100.0 + SyncExtra; Syncing := TRUE; Init := TRUE;
END_IF;
IF Open THEN Pi := Pi + Rate * T.Dt; ELSIF Close THEN Pi := Pi - Rate * T.Dt; END_IF;
IF Open OR Close THEN Moving := Moving + T.DtT; END_IF;
IF Syncing THEN
    Target := -SyncExtra;
ELSIF Y >= 100.0 THEN
    IF EndHold = 1 THEN Target := Pi; ELSE Target := 100.0 + SyncExtra; END_IF;
ELSIF Y <= 0.0 THEN
    IF EndHold = -1 THEN Target := Pi; ELSE Target := -SyncExtra; END_IF;
ELSE
    Target := Y; EndHold := 0;
END_IF;
IF Open THEN
    IF Pi >= Target AND Moving >= MinPulse THEN Open := FALSE; END_IF;
ELSIF Close THEN
    IF Pi <= Target AND Moving >= MinPulse THEN Close := FALSE; END_IF;
ELSIF Target - Pi > Deadband THEN
    Open := TRUE; Moving := T#0s;
ELSIF Pi - Target > Deadband THEN
    Close := TRUE; Moving := T#0s;
END_IF;
IF NOT Open AND NOT Close THEN
    IF Pi > 100.0 THEN Pi := 100.0; EndHold := 1; END_IF;
    IF Pi < 0.0 THEN Pi := 0.0; EndHold := -1; Syncing := FALSE; END_IF;
END_IF;
Pos := LIMIT(0.0, Pi, 100.0);
END_FUNCTION_BLOCK

FUNCTION_BLOCK OUT3PVAR
(* Stellausgabe 3-Punkt mit variabler Impulslänge: Regelabweichung -> Auf-/Zu-Impulse (Schrittregler)
   Every Period one pulse in the direction of E (positive: open) of the length |E| * Kp % of RunTime,
   at least MinPulse, at most Period; none within the Deadband. *)
VAR_INPUT E : REAL; Kp : REAL := 1.0; RunTime : TIME := T#120s; Period : TIME := T#10s; MinPulse : TIME := T#200ms; Deadband : REAL := 0.5; END_VAR
VAR_OUTPUT Open : BOOL; Close : BOOL; Pulse : TIME; END_VAR
VAR T : __Dt; Phase : TIME; Dir : INT; Init : BOOL; Len : REAL; END_VAR
T();
IF Init THEN Phase := Phase + T.DtT; END_IF;
IF NOT Init OR Phase >= Period THEN
    Init := TRUE;
    Phase := T#0s;
    IF ABS(E) <= Deadband THEN
        Pulse := T#0s; Dir := 0;
    ELSE
        Len := LIMIT(TIME_TO_REAL(MinPulse), ABS(E) * Kp / 100.0 * TIME_TO_REAL(RunTime), TIME_TO_REAL(Period));
        Pulse := REAL_TO_TIME(Len);
        IF E > 0.0 THEN Dir := 1; ELSE Dir := -1; END_IF;
    END_IF;
END_IF;
Open := Dir > 0 AND Phase < Pulse;
Close := Dir < 0 AND Phase < Pulse;
END_FUNCTION_BLOCK

FUNCTION_BLOCK PWM
(* Stellausgabe Pulsweitenmodulation: Stellgröße 0..100 % als Einschaltanteil einer Periode
   OnTime = Y % of Period, taken at the start of each period; shorter than MinPulse: off, less than
   MinPulse before the end: on for the whole period. *)
VAR_INPUT Y : REAL; Period : TIME := T#10s; MinPulse : TIME; Enable : BOOL := TRUE; END_VAR
VAR_OUTPUT Q : BOOL; OnTime : TIME; END_VAR
VAR T : __Dt; Phase : TIME; Init : BOOL; END_VAR
T();
IF Init THEN Phase := Phase + T.DtT; END_IF;
IF NOT Init OR Phase >= Period THEN
    Init := TRUE;
    Phase := T#0s;
    OnTime := REAL_TO_TIME(LIMIT(0.0, Y, 100.0) / 100.0 * TIME_TO_REAL(Period));
    IF OnTime < MinPulse THEN OnTime := T#0s; END_IF;
    IF Period - OnTime < MinPulse THEN OnTime := Period; END_IF;
END_IF;
Q := Enable AND Phase < OnTime;
END_FUNCTION_BLOCK

FUNCTION_BLOCK LIMITSP
(* Begrenzung Sollwert/Stellwert: Grenzen und Änderungsgeschwindigkeit
   Out = In within Lo..Hi, changing at most RateUp / RateDown [unit/s] (0 = unlimited). *)
VAR_INPUT In : REAL; Lo : REAL := -1.0E30; Hi : REAL := 1.0E30; RateUp : REAL; RateDown : REAL; END_VAR
VAR_OUTPUT Out : REAL; AtLo : BOOL; AtHi : BOOL; Limited : BOOL; END_VAR
VAR T : __Dt; Target : REAL; Init : BOOL; END_VAR
T();
Target := LIMIT(Lo, In, Hi);
AtLo := In < Lo; AtHi := In > Hi;
IF NOT Init THEN
    Out := Target; Init := TRUE;
ELSE
    IF RateUp > 0.0 AND Target > Out + RateUp * T.Dt THEN
        Out := Out + RateUp * T.Dt;
    ELSIF RateDown > 0.0 AND Target < Out - RateDown * T.Dt THEN
        Out := Out - RateDown * T.Dt;
    ELSE
        Out := Target;
    END_IF;
END_IF;
Limited := Out <> In;
END_FUNCTION_BLOCK

FUNCTION_BLOCK PARAMSET
(* Parameterumschaltung: einen von 4 Parametersätzen (je 3 Werte, z. B. Kp, Tn, Tv) wählen *)
VAR_INPUT Sel : INT := 1; A1 : REAL; B1 : REAL; C1 : REAL; A2 : REAL; B2 : REAL; C2 : REAL;
    A3 : REAL; B3 : REAL; C3 : REAL; A4 : REAL; B4 : REAL; C4 : REAL; END_VAR
VAR_OUTPUT A : REAL; B : REAL; C : REAL; END_VAR
CASE Sel OF
    2: A := A2; B := B2; C := C2;
    3: A := A3; B := B3; C := C3;
    4: A := A4; B := B4; C := C4;
ELSE
    A := A1; B := B1; C := C1;
END_CASE;
END_FUNCTION_BLOCK
`,
	},
	{
		group: "VDI 3814 · 2.4 Optimierung",
		source: `
FUNCTION_BLOCK ENERGYLEVEL
(* Energieniveau: Komfort, Pre-Komfort, Economy, Gebäudeschutz -> Heiz- und Kühlsollwert
   Level 1 comfort, 2 pre-comfort (standby), 3 economy (night), 4 building protection. Presence raises
   pre-comfort to comfort; an open Window means building protection. The levels widen the comfort band
   by PreShift / EcoShift on both sides; Shift (user setpoint correction) moves comfort and pre-comfort. *)
VAR_INPUT Level : INT := 1; Presence : BOOL; Window : BOOL; HeatComfort : REAL := 21.0; CoolComfort : REAL := 24.0;
    PreShift : REAL := 2.0; EcoShift : REAL := 4.0; ProtHeat : REAL := 7.0; ProtCool : REAL := 35.0; Shift : REAL; END_VAR
VAR_OUTPUT HeatSP : REAL; CoolSP : REAL; Active : INT; END_VAR
Active := LIMIT(1, Level, 4);
IF Window THEN Active := 4; ELSIF Presence AND Active = 2 THEN Active := 1; END_IF;
CASE Active OF
    1: HeatSP := HeatComfort + Shift; CoolSP := CoolComfort + Shift;
    2: HeatSP := HeatComfort + Shift - PreShift; CoolSP := CoolComfort + Shift + PreShift;
    3: HeatSP := HeatComfort - EcoShift; CoolSP := CoolComfort + EcoShift;
ELSE
    HeatSP := ProtHeat; CoolSP := ProtCool;
END_CASE;
END_FUNCTION_BLOCK

FUNCTION_BLOCK HEATRECOVERY
(* Energierückgewinnung: Wärmerückgewinnung nach Bedarf aus Außen-, Abluft- und Zuluftsollwert
   Heating: SupplySP above Toa and Tex above Toa; cooling: the other way round. Y [%] is what the
   recovery (temperature efficiency Efficiency at 100 %) needs to reach SupplySP; Tsupply what comes out.
   Below IcingLimit outdoor temperature Y is at most IcingY (icing protection). *)
VAR_INPUT Toa : REAL; Tex : REAL; SupplySP : REAL; Efficiency : REAL := 0.7; IcingLimit : REAL := -5.0; IcingY : REAL := 50.0; Enable : BOOL := TRUE; END_VAR
VAR_OUTPUT Y : REAL; Heating : BOOL; Cooling : BOOL; Tsupply : REAL; END_VAR
VAR Eff : REAL; END_VAR
Eff := LIMIT(0.05, Efficiency, 1.0);
Heating := Enable AND SupplySP > Toa AND Tex > Toa;
Cooling := Enable AND SupplySP < Toa AND Tex < Toa;
IF Heating THEN
    Y := LIMIT(0.0, (SupplySP - Toa) / (Eff * (Tex - Toa)) * 100.0, 100.0);
ELSIF Cooling THEN
    Y := LIMIT(0.0, (Toa - SupplySP) / (Eff * (Toa - Tex)) * 100.0, 100.0);
ELSE
    Y := 0.0;
END_IF;
IF Toa < IcingLimit THEN Y := MIN(Y, IcingY); END_IF;
Tsupply := Toa + Y / 100.0 * Eff * (Tex - Toa);
END_FUNCTION_BLOCK

FUNCTION_BLOCK HXSETPOINT
(* h,x-Sollwertführung: Wassergehalt, Enthalpie, Taupunkt; Be-/Entfeuchten außerhalb des Behaglichkeitsfelds
   From T [°C] and RH [%] (Magnus formula over water, Pressure [hPa]): X [g/kg], H [kJ/kg], DewPoint
   [°C]. XSet is X limited to XMin..XMax (the comfort field: humidify or dehumidify only outside it,
   with Hyst g/kg); RHSet the relative humidity of XSet at T. *)
VAR_INPUT T : REAL; RH : REAL; Pressure : REAL := 1013.25; XMin : REAL := 6.0; XMax : REAL := 11.5; Hyst : REAL := 0.3; END_VAR
VAR_OUTPUT X : REAL; H : REAL; DewPoint : REAL; XSet : REAL; RHSet : REAL; Humidify : BOOL; Dehumidify : BOOL; END_VAR
VAR Ps : REAL; Pw : REAL; G : REAL; PwSet : REAL; END_VAR
Ps := 6.112 * EXP(17.62 * T / (243.12 + T));
Pw := LIMIT(0.001, RH, 100.0) / 100.0 * Ps;
X := 622.0 * Pw / (Pressure - Pw);
H := 1.006 * T + X / 1000.0 * (2501.0 + 1.86 * T);
G := LN(Pw / 6.112);
DewPoint := 243.12 * G / (17.62 - G);
XSet := LIMIT(XMin, X, XMax);
PwSet := XSet * Pressure / (622.0 + XSet);
RHSet := PwSet / Ps * 100.0;
IF X < XMin - Hyst THEN Humidify := TRUE; ELSIF X >= XMin THEN Humidify := FALSE; END_IF;
IF X > XMax + Hyst THEN Dehumidify := TRUE; ELSIF X <= XMax THEN Dehumidify := FALSE; END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK OPTSTART
(* Schaltzeitpunktoptimierung: so spät wie möglich vor Nutzungsbeginn aufheizen, Aufheizrate lernen
   Lead = (SP - Troom) / rate (at most MaxLead); Start is TRUE from OccStart - Lead until OccStart
   (TimeOfDay from VDI3814.CLOCK). The rate [K/h] starts with Rate and, with Learn, is corrected after each
   heat-up that reached SP (LearnedRate). *)
VAR_INPUT TimeOfDay : TIME; OccStart : TIME := T#7h; Troom : REAL; SP : REAL := 21.0; Rate : REAL := 2.0; MaxLead : TIME := T#3h; Learn : BOOL := TRUE; END_VAR
VAR_OUTPUT Start : BOOL; Lead : TIME; LearnedRate : REAL; END_VAR
VAR Remaining : TIME; Init : BOOL; TStart : REAL; Began : TIME; Measuring : BOOL; Hours : REAL; LeadS : REAL; Last : BOOL; END_VAR
IF NOT Init THEN LearnedRate := MAX(Rate, 0.1); Init := TRUE; END_IF;
LeadS := LIMIT(0.0, (SP - Troom) / MAX(LearnedRate, 0.1) * 3600.0, TIME_TO_REAL(MaxLead) * 0.001);
Lead := REAL_TO_TIME(LeadS * 1000.0);
Remaining := OccStart - TimeOfDay;
IF Remaining < T#0s THEN Remaining := Remaining + T#24h; END_IF;
IF NOT Start THEN
    Start := Remaining > T#0s AND Remaining <= Lead AND Troom < SP;
ELSIF Remaining = T#0s OR Remaining > MaxLead THEN
    Start := FALSE;
END_IF;
IF Start AND NOT Last THEN
    Measuring := TRUE; TStart := Troom; Began := TimeOfDay;
END_IF;
IF Measuring AND Troom >= SP THEN
    Measuring := FALSE;
    Hours := TIME_TO_REAL(TimeOfDay - Began) / 3600000.0;
    IF Hours < 0.0 THEN Hours := Hours + 24.0; END_IF;
    IF Learn AND Hours > 0.05 AND SP > TStart THEN
        LearnedRate := LIMIT(0.1, 0.7 * LearnedRate + 0.3 * (SP - TStart) / Hours, 50.0);
    END_IF;
END_IF;
IF Measuring AND NOT Start AND Remaining > MaxLead THEN Measuring := FALSE; END_IF;
Last := Start;
END_FUNCTION_BLOCK

FUNCTION_BLOCK NIGHTCOOL
(* Nachtkühlung: Gebäude nachts mit kühler Außenluft spülen
   Q when Night, the room warmer than TroomStart and DeltaT (+Hyst) warmer than outside, outside not
   colder than ToaMin; off when the room is below TroomMin, the difference below DeltaT, outside below
   ToaMin - Hyst, or the night is over. *)
VAR_INPUT Night : BOOL; Troom : REAL; Toa : REAL; TroomStart : REAL := 23.0; TroomMin : REAL := 18.0; DeltaT : REAL := 3.0; ToaMin : REAL := 10.0; Hyst : REAL := 1.0; END_VAR
VAR_OUTPUT Q : BOOL; END_VAR
IF NOT Q THEN
    Q := Night AND Troom > TroomStart AND Troom - Toa > DeltaT + Hyst AND Toa > ToaMin;
ELSIF NOT Night OR Troom < TroomMin OR Troom - Toa < DeltaT OR Toa < ToaMin - Hyst THEN
    Q := FALSE;
END_IF;
END_FUNCTION_BLOCK
`,
	},
	{
		group: "VDI 3814 · 2.5 Beleuchtung",
		source: `
FUNCTION_BLOCK LIGHT
(* Lichtsteuerung: Taster kurz = Ein/Aus, lang = Dimmen; zentral Ein/Aus; Abwesenheit schaltet aus
   A press shorter than LongPress toggles; held longer it dims (DimSpeed %/s, MinLevel..100 %): off it
   switches on at MinLevel and dims up; at 100 % down, at MinLevel up, in between the opposite of the last
   long press. On / Off (rising edges) switch centrally (On: OnLevel). Without
   Presence for AbsenceDelay the light goes off. *)
VAR_INPUT Button : BOOL; On : BOOL; Off : BOOL; Presence : BOOL := TRUE; AbsenceDelay : TIME := T#15m; LongPress : TIME := T#500ms;
    DimSpeed : REAL := 25.0; MinLevel : REAL := 10.0; OnLevel : REAL := 100.0; END_VAR
VAR_OUTPUT Q : BOOL; Level : REAL; END_VAR
VAR T : __Dt; Held : TIME; Dimming : BOOL; DimUp : BOOL; Last : BOOL; OnE : R_TRIG; OffE : R_TRIG; Away : TON; END_VAR
T();
OnE(CLK := On);
OffE(CLK := Off);
IF Button THEN
    Held := Held + T.DtT;
    IF Held >= LongPress THEN
        IF NOT Dimming THEN
            Dimming := TRUE;
            IF NOT Q THEN
                Q := TRUE; Level := MinLevel; DimUp := TRUE;
            ELSIF Level >= 100.0 THEN
                DimUp := FALSE;
            ELSIF Level <= MinLevel THEN
                DimUp := TRUE;
            ELSE
                DimUp := NOT DimUp;
            END_IF;
        END_IF;
        IF DimUp THEN Level := Level + DimSpeed * T.Dt; ELSE Level := Level - DimSpeed * T.Dt; END_IF;
        Level := LIMIT(MinLevel, Level, 100.0);
    END_IF;
ELSIF Last THEN
    IF NOT Dimming THEN
        Q := NOT Q;
        IF Q THEN Level := OnLevel; END_IF;
    END_IF;
    Held := T#0s; Dimming := FALSE;
END_IF;
Last := Button;
IF OnE.Q THEN Q := TRUE; Level := OnLevel; END_IF;
IF OffE.Q THEN Q := FALSE; END_IF;
Away(IN := NOT Presence, PT := AbsenceDelay);
IF Away.Q THEN Q := FALSE; END_IF;
IF NOT Q THEN Level := 0.0; END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK STAIRLIGHT
(* Treppenlichtschaltung: Licht für eine Zeit, nachtriggerbar, Ausschaltvorwarnung, Dauerlicht
   A rising edge of Trigger switches on for Duration (again from the start). Warn before the end the
   light blinks off once (500 ms) and Level drops to 50 %. A press held for LongPress switches to
   permanent light; the next press ends it. *)
VAR_INPUT Trigger : BOOL; Duration : TIME := T#3m; Warn : TIME := T#30s; LongPress : TIME := T#2s; END_VAR
VAR_OUTPUT Q : BOOL; Warning : BOOL; Permanent : BOOL; Remaining : TIME; Level : REAL; END_VAR
VAR T : __Dt; Edge : R_TRIG; Held : TIME; Since : TIME; Running : BOOL; END_VAR
T();
Edge(CLK := Trigger);
IF Trigger THEN Held := Held + T.DtT; ELSE Held := T#0s; END_IF;
IF Edge.Q THEN
    IF Permanent THEN Permanent := FALSE; Running := FALSE; ELSE Running := TRUE; Since := T#0s; END_IF;
END_IF;
IF Trigger AND Held >= LongPress AND Held - T.DtT < LongPress THEN Permanent := TRUE; END_IF;
IF Running THEN
    Since := Since + T.DtT;
    IF Since >= Duration THEN Running := FALSE; END_IF;
END_IF;
IF Running THEN Remaining := Duration - Since; ELSE Remaining := T#0s; END_IF;
Warning := Running AND NOT Permanent AND Remaining <= Warn;
Q := Permanent OR (Running AND NOT (Warning AND Remaining > Warn - T#500ms));
IF NOT Q THEN Level := 0.0; ELSIF Warning THEN Level := 50.0; ELSE Level := 100.0; END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK DAYLIGHT
(* Tageslichtschaltung: Licht anfordern, wenn es zu dunkel ist (Schaltdifferenz, Verzögerung)
   Q goes on when Lux stayed below OnBelow for Delay, off when it stayed above OffAbove for Delay. *)
VAR_INPUT Lux : REAL; OnBelow : REAL := 300.0; OffAbove : REAL := 500.0; Delay : TIME := T#1m; END_VAR
VAR_OUTPUT Q : BOOL; END_VAR
VAR A : TON; B : TON; END_VAR
A(IN := NOT Q AND Lux < OnBelow, PT := Delay);
B(IN := Q AND Lux > OffAbove, PT := Delay);
IF A.Q THEN Q := TRUE; ELSIF B.Q THEN Q := FALSE; END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK CONSTLIGHT
(* Konstantlichtregelung: Kunstlicht so dimmen, dass die Beleuchtungsstärke den Sollwert hält
   I controller: Level [%] += Ki * (SP - Lux) * dt, MinLevel..100 %. When it stays at MinLevel with
   more than 110 % of SP for OffDelay the light goes off; below 90 % of SP it comes on again. *)
VAR_INPUT Enable : BOOL := TRUE; Lux : REAL; SP : REAL := 500.0; Ki : REAL := 0.02; MinLevel : REAL := 10.0; OffDelay : TIME := T#5m; END_VAR
VAR_OUTPUT Q : BOOL; Level : REAL; END_VAR
VAR T : __Dt; Bright : TON; END_VAR
T();
IF NOT Enable THEN
    Q := FALSE;
ELSE
    IF NOT Q AND Lux < 0.9 * SP THEN Q := TRUE; Level := MinLevel; END_IF;
    IF Q THEN
        Level := LIMIT(MinLevel, Level + Ki * (SP - Lux) * T.Dt, 100.0);
    END_IF;
    Bright(IN := Q AND Level <= MinLevel AND Lux > 1.1 * SP, PT := OffDelay);
    IF Bright.Q THEN Q := FALSE; END_IF;
END_IF;
IF NOT Q THEN Level := 0.0; END_IF;
END_FUNCTION_BLOCK
`,
	},
	{
		group: "VDI 3814 · 2.6 Sonnenschutz",
		source: `
FUNCTION_BLOCK SUNBLIND
(* Sonnenschutz stellen: Höhe und Lamelle nach Priorität (Schutz > Hand > Automatik), Antrieb Auf/Ab
   Positions in % (0 = up/open, 100 = down/closed), slats 0 = open, 100 = closed. Protect: ProtPos;
   a rising edge of ManSet takes ManPos/ManSlat for ManTimeout (0: until the next automatic change);
   else AutoPos/AutoSlat. The drive model (TravelTime for the height, SlatTime to turn the slats) gives
   Up/Down and the actual ActPos/ActSlat: moving down first closes the slats, moving up opens them. *)
VAR_INPUT AutoPos : REAL; AutoSlat : REAL; ManPos : REAL; ManSlat : REAL; ManSet : BOOL; ManTimeout : TIME := T#2h;
    Protect : BOOL; ProtPos : REAL; TravelTime : TIME := T#60s; SlatTime : TIME := T#1500ms; END_VAR
VAR_OUTPUT Pos : REAL; Slat : REAL; Up : BOOL; Down : BOOL; ActPos : REAL; ActSlat : REAL; Mode : INT; END_VAR
VAR T : __Dt; MS : R_TRIG; Manual : BOOL; ManFor : TIME; LastAutoPos : REAL; LastAutoSlat : REAL; Rp : REAL; Rs : REAL; END_VAR
T();
MS(CLK := ManSet);
IF MS.Q THEN Manual := TRUE; ManFor := T#0s; END_IF;
IF Manual THEN
    ManFor := ManFor + T.DtT;
    IF ManTimeout > T#0s AND ManFor >= ManTimeout THEN Manual := FALSE; END_IF;
    IF ManTimeout = T#0s AND (AutoPos <> LastAutoPos OR AutoSlat <> LastAutoSlat) AND NOT MS.Q THEN Manual := FALSE; END_IF;
END_IF;
LastAutoPos := AutoPos; LastAutoSlat := AutoSlat;
IF Protect THEN
    Mode := 2; Pos := ProtPos; Slat := 0.0;
ELSIF Manual THEN
    Mode := 1; Pos := ManPos; Slat := ManSlat;
ELSE
    Mode := 0; Pos := AutoPos; Slat := AutoSlat;
END_IF;
Pos := LIMIT(0.0, Pos, 100.0);
Slat := LIMIT(0.0, Slat, 100.0);
Rp := 100.0 / MAX(TIME_TO_REAL(TravelTime) * 0.001, 0.1) * T.Dt;
Rs := 100.0 / MAX(TIME_TO_REAL(SlatTime) * 0.001, 0.05) * T.Dt;
Down := FALSE; Up := FALSE;
IF Pos - ActPos > 0.5 THEN
    Down := TRUE;
ELSIF ActPos - Pos > 0.5 THEN
    Up := TRUE;
ELSIF ActPos > 0.5 THEN
    IF Slat - ActSlat > 1.0 THEN Down := TRUE; ELSIF ActSlat - Slat > 1.0 THEN Up := TRUE; END_IF;
END_IF;
IF Down THEN
    IF ActSlat < 100.0 AND (ActPos <= Pos - 0.5 OR ActSlat < Slat) THEN
        ActSlat := MIN(ActSlat + Rs, 100.0);
    ELSE
        ActPos := MIN(ActPos + Rp, Pos);
    END_IF;
ELSIF Up THEN
    IF ActSlat > 0.0 AND (ActPos >= Pos + 0.5 OR ActSlat > Slat) THEN
        ActSlat := MAX(ActSlat - Rs, 0.0);
    ELSE
        ActPos := MAX(ActPos - Rp, Pos);
    END_IF;
END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SUNDUSK
(* Sonnen-/Dämmerungsautomatik: Sonne (Beschattung anfordern) und Dämmerung aus der Außenhelligkeit
   Sun after Lux stayed above SunOn for SunOnDelay, until it stayed below SunOff for SunOffDelay; Dusk
   after Lux stayed below DuskBelow for DuskDelay, until it stayed above DuskAbove as long. *)
VAR_INPUT Lux : REAL; SunOn : REAL := 30000.0; SunOff : REAL := 20000.0; SunOnDelay : TIME := T#2m; SunOffDelay : TIME := T#15m;
    DuskBelow : REAL := 50.0; DuskAbove : REAL := 100.0; DuskDelay : TIME := T#5m; END_VAR
VAR_OUTPUT Sun : BOOL; Dusk : BOOL; END_VAR
VAR A : TON; B : TON; C : TON; D : TON; END_VAR
A(IN := NOT Sun AND Lux > SunOn, PT := SunOnDelay);
B(IN := Sun AND Lux < SunOff, PT := SunOffDelay);
IF A.Q THEN Sun := TRUE; ELSIF B.Q THEN Sun := FALSE; END_IF;
C(IN := NOT Dusk AND Lux < DuskBelow, PT := DuskDelay);
D(IN := Dusk AND Lux > DuskAbove, PT := DuskDelay);
IF C.Q THEN Dusk := TRUE; ELSIF D.Q THEN Dusk := FALSE; END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK THERMOAUTO
(* Thermoautomatik: bei Abwesenheit Sonnenschutz zum Kühlen schließen oder zum solaren Heizen öffnen
   Only when not Occupied and Sun: Troom above CoolLimit -> Close (Pos 100 %), below HeatLimit -> Open
   (Pos 0 %), with Hyst. Active: one of them; use Pos as a priority level of SUNBLIND/PRIORITY. *)
VAR_INPUT Occupied : BOOL; Sun : BOOL; Troom : REAL; HeatLimit : REAL := 20.0; CoolLimit : REAL := 25.0; Hyst : REAL := 0.5; END_VAR
VAR_OUTPUT Active : BOOL; Close : BOOL; Open : BOOL; Pos : REAL; END_VAR
IF Occupied OR NOT Sun THEN
    Close := FALSE; Open := FALSE;
ELSE
    IF Troom > CoolLimit THEN Close := TRUE; ELSIF Troom < CoolLimit - Hyst THEN Close := FALSE; END_IF;
    IF Troom < HeatLimit THEN Open := TRUE; ELSIF Troom > HeatLimit + Hyst THEN Open := FALSE; END_IF;
END_IF;
Active := Close OR Open;
IF Close THEN Pos := 100.0; ELSE Pos := 0.0; END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK WEATHERPROT
(* Witterungsschutz: Wind, Regen und Frost -> Schutzstellung mit Rückfallverzögerung
   WindAlarm above WindLimit [m/s] (until below WindLimit - WindHyst), RainAlarm while Rain, FrostAlarm
   when Toa < FrostLimit after precipitation (it stays wet until Toa > FrostLimit + 2 K). Protect (rain
   only with RainProtect, e.g. awnings) is released Release after the last alarm ended. *)
VAR_INPUT Wind : REAL; WindLimit : REAL := 10.0; WindHyst : REAL := 3.0; Rain : BOOL; RainProtect : BOOL; Toa : REAL := 10.0; FrostLimit : REAL := 2.0; Release : TIME := T#10m; END_VAR
VAR_OUTPUT Protect : BOOL; WindAlarm : BOOL; RainAlarm : BOOL; FrostAlarm : BOOL; END_VAR
VAR Wet : BOOL; Hold : TOF; END_VAR
IF Wind > WindLimit THEN WindAlarm := TRUE; ELSIF Wind < WindLimit - WindHyst THEN WindAlarm := FALSE; END_IF;
RainAlarm := Rain;
IF Rain THEN Wet := TRUE; ELSIF Toa > FrostLimit + 2.0 THEN Wet := FALSE; END_IF;
FrostAlarm := Wet AND Toa < FrostLimit;
Hold(IN := WindAlarm OR (RainProtect AND RainAlarm) OR FrostAlarm, PT := Release);
Protect := Hold.Q;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SUNPOS
(* Sonnenstandsberechnung: Höhe und Azimut der Sonne aus Tag, Uhrzeit und Ort
   NOAA approximation (about 0.5°): DayOfYear 1..366, TimeOfDay local time, UtcOffset [h] (1 = CET,
   2 = CEST), Latitude/Longitude [°] (north/east positive). Elevation [°] above the horizon, Azimuth [°]
   from north over east (90 east, 180 south, 270 west); Day: the sun is above the horizon. *)
VAR_INPUT DayOfYear : INT := 172; TimeOfDay : TIME := T#12h; UtcOffset : REAL := 1.0; Latitude : REAL := 51.0; Longitude : REAL := 10.0; END_VAR
VAR_OUTPUT Elevation : REAL; Azimuth : REAL; Day : BOOL; END_VAR
VAR G : REAL; Eqt : REAL; Decl : REAL; Hours : REAL; Tst : REAL; Ha : REAL; Lat : REAL; CosZ : REAL; Zen : REAL; END_VAR
Hours := TIME_TO_REAL(TimeOfDay) / 3600000.0;
G := 2.0 * 3.14159265 / 365.0 * (INT_TO_REAL(DayOfYear) - 1.0 + (Hours - UtcOffset - 12.0) / 24.0);
Eqt := 229.18 * (0.000075 + 0.001868 * COS(G) - 0.032077 * SIN(G) - 0.014615 * COS(2.0 * G) - 0.040849 * SIN(2.0 * G));
Decl := 0.006918 - 0.399912 * COS(G) + 0.070257 * SIN(G) - 0.006758 * COS(2.0 * G) + 0.000907 * SIN(2.0 * G)
    - 0.002697 * COS(3.0 * G) + 0.00148 * SIN(3.0 * G);
Tst := Hours * 60.0 + Eqt + 4.0 * Longitude - 60.0 * UtcOffset;
Ha := (Tst / 4.0 - 180.0) * 3.14159265 / 180.0;
Lat := Latitude * 3.14159265 / 180.0;
CosZ := LIMIT(-1.0, SIN(Lat) * SIN(Decl) + COS(Lat) * COS(Decl) * COS(Ha), 1.0);
Zen := ACOS(CosZ);
Elevation := 90.0 - Zen * 180.0 / 3.14159265;
Azimuth := __Atan2(SIN(Ha), COS(Ha) * SIN(Lat) - SIN(Decl) / COS(Decl) * COS(Lat)) * 180.0 / 3.14159265 + 180.0;
Day := Elevation > 0.0;
END_FUNCTION_BLOCK
`,
	},
];

/** The library's source: the namespace with the helpers and every section. */
export function vdi3814LibSource(): string {
	return `NAMESPACE ${VDI3814_NAMESPACE}\n${HELPERS}\n${SECTIONS.map((s) => s.source).join("\n")}\nEND_NAMESPACE\n`;
}

/** The library's sections (palette groups), each as a text the FBD palette reads its boxes from. */
export function vdi3814Sections(): readonly Section[] {
	return SECTIONS;
}
