// The simulation library ("Realsimulatoren"): function blocks that map physical quantities at their inputs
// to physical quantities at their outputs with the dynamics of the real equipment - motors, compressors,
// pressure vessels, lead screws, heating elements, heat transfer. In the namespace SIM (`SIM.MOTOR`, or
// `USING SIM;`); they run on the PLC like any block, so a program can simulate its plant in the browser
// simulator and on the FCU itself (virtual commissioning, teaching without hardware).
//
// Every block takes its time step from the PLC time (__PLC_TIME) and integrates with sub-steps or the exact
// solution of its linear part, so it stays stable for any cycle time (at most 1 s per call is integrated:
// a halted program does not jump). SI units unless a comment says otherwise; temperatures in °C,
// pressures in bar gauge, air volumes as normal litres [Nl] (1.01325 bar, 0 °C).
//
// Coupled blocks (heater -> heat transfer -> thermal mass) exchange their values once per cycle; that is
// accurate while each body's time constant is long against the cycle (C / UA >> cycle time).

export const SIM_LIB_FILE = "<sim>";
export const SIM_NAMESPACE = "SIM";

interface Section {
	group: string;
	source: string;
}

const HELPERS = `
FUNCTION_BLOCK __Dt
(* Zeitschritt seit dem letzten Aufruf *)
VAR_OUTPUT Dt : REAL; (* [s], 0 at the first call, at most 1 s *) DtT : TIME; END_VAR
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

FUNCTION __Steps : INT
(* Teilschritte, damit Dt * Rate je Teilschritt höchstens 0,2 ist (1..200) *)
VAR_INPUT Dt : REAL; Rate : REAL; END_VAR
__Steps := REAL_TO_INT(LIMIT(1.0, Dt * ABS(Rate) / 0.2 + 0.5, 200.0));
END_FUNCTION
`;

const SECTIONS: Section[] = [
	{
		group: "Simulation · Antriebe",
		source: `
FUNCTION_BLOCK MOTOR
(* Motor: Drehzahl, Drehmoment, Strom aus Spannung und Lastmoment
   A DC motor (and an induction motor on a frequency converter in its working range): the torque falls
   linearly from StallTorque at standstill to 0 at NoLoadSpeed, both scaled by U (% of the rated
   voltage/frequency, negative: reverse). On FALSE: no torque, the motor coasts. LoadTorque [Nm] is a
   passive load (friction, process): it brakes the motion and holds a standstill while the motor torque is
   smaller. Friction: viscous loss [Nm per 1000 rpm]; Inertia [kg m2] of motor and load; Brake holds
   the shaft. Current proportional to the torque (StallCurrent at stall), plus NoLoadCurrent while turning. *)
VAR_INPUT
    U : REAL := 100.0; On : BOOL; LoadTorque : REAL; Brake : BOOL;
    NoLoadSpeed : REAL := 3000.0; StallTorque : REAL := 2.0; StallCurrent : REAL := 10.0; NoLoadCurrent : REAL := 0.3;
    Inertia : REAL := 0.0005; Friction : REAL := 0.01;
END_VAR
VAR_OUTPUT Speed : REAL; (* [rpm] *) Torque : REAL; (* motor torque [Nm] *) Current : REAL; (* [A] *) Power : REAL; (* shaft [W] *) Revs : LREAL; END_VAR
VAR T : __Dt; N : INT; I : INT; H : REAL; K : REAL; Drive : REAL; Damp : REAL; Tl : REAL; Nn : REAL; Uf : REAL; N0 : REAL; END_VAR
T();
N0 := MAX(NoLoadSpeed, 1.0);
IF On THEN Uf := LIMIT(-100.0, U, 100.0) / 100.0; ELSE Uf := 0.0; END_IF;
K := 9.549297 / MAX(Inertia, 1.0E-6);          (* rpm/s per Nm *)
Damp := Friction / 1000.0;
IF On THEN Damp := Damp + StallTorque / N0; END_IF;
Drive := StallTorque * Uf;
N := __Steps(T.Dt, K * Damp);
H := T.Dt / INT_TO_REAL(N);
FOR I := 1 TO N DO
    IF Brake THEN
        Speed := 0.0;
    ELSE
        IF Speed > 0.0 THEN
            Tl := ABS(LoadTorque);
        ELSIF Speed < 0.0 THEN
            Tl := -ABS(LoadTorque);
        ELSIF ABS(Drive) <= ABS(LoadTorque) THEN
            Tl := Drive;                          (* static: the load holds it *)
        ELSIF Drive > 0.0 THEN
            Tl := ABS(LoadTorque);
        ELSE
            Tl := -ABS(LoadTorque);
        END_IF;
        Nn := (Speed + H * K * (Drive - Tl)) / (1.0 + H * K * Damp);   (* implicit Euler *)
        IF (Speed > 0.0 AND Nn < 0.0) OR (Speed < 0.0 AND Nn > 0.0) THEN Nn := 0.0; END_IF;
        Speed := Nn;
    END_IF;
    Revs := Revs + Speed / 60.0 * H;
END_FOR;
IF On THEN Torque := StallTorque * (Uf - Speed / N0); ELSE Torque := 0.0; END_IF;
Current := Torque / MAX(StallTorque, 1.0E-6) * StallCurrent;
IF Speed <> 0.0 THEN
    IF Current >= 0.0 THEN Current := Current + NoLoadCurrent; ELSE Current := Current - NoLoadCurrent; END_IF;
END_IF;
IF NOT On THEN Current := 0.0; END_IF;
Power := Torque * Speed * 0.10471976;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SPINDLE
(* Drehspindel: Drehbewegung in einen Weg integrieren (Gewindespindel mit Endanschlägen)
   Speed [rpm] (e.g. SIM.MOTOR.Speed) through the gear Ratio (motor : spindle) and the Pitch [mm/rev]
   gives Velocity [mm/s]; Pos [mm] integrates it between the end stops MinPos/MaxPos. The limit
   switches AtMin/AtMax switch LimitWindow before the stops; Blocked: driven against a stop (feed it back
   to the motor's Brake or LoadTorque). A rising edge of Set takes SetPos (homing); P0: the start
   position. *)
VAR_INPUT Speed : REAL; Ratio : REAL := 1.0; Pitch : REAL := 4.0; MinPos : REAL := 0.0; MaxPos : REAL := 300.0; LimitWindow : REAL := 1.0;
    Set : BOOL; SetPos : REAL; P0 : REAL; END_VAR
VAR_OUTPUT Pos : REAL; (* [mm] *) Velocity : REAL; (* [mm/s] *) AtMin : BOOL; AtMax : BOOL; Blocked : BOOL; Revs : LREAL; (* spindle revolutions *) END_VAR
VAR T : __Dt; S : R_TRIG; Init : BOOL; P : LREAL; END_VAR
T();
S(CLK := Set);
IF NOT Init THEN P := P0; Init := TRUE; END_IF;
IF S.Q THEN P := SetPos; END_IF;
Velocity := Speed / MAX(ABS(Ratio), 1.0E-6) / 60.0 * Pitch;
IF Ratio < 0.0 THEN Velocity := -Velocity; END_IF;
Blocked := (P <= MinPos AND Velocity < 0.0) OR (P >= MaxPos AND Velocity > 0.0);
IF NOT Blocked THEN
    P := P + Velocity * T.Dt;
    Revs := Revs + Speed / MAX(ABS(Ratio), 1.0E-6) / 60.0 * T.Dt;
END_IF;
IF P < MinPos THEN P := MinPos; END_IF;
IF P > MaxPos THEN P := MaxPos; END_IF;
IF Blocked THEN Velocity := 0.0; END_IF;
Pos := LREAL_TO_REAL(P);
AtMin := P <= MinPos + LimitWindow;
AtMax := P >= MaxPos - LimitWindow;
END_FUNCTION_BLOCK

FUNCTION_BLOCK ACTUATOR
(* Stellantrieb: Ventil-/Klappenantrieb mit Laufzeit, 3-Punkt (Auf/Zu) oder stetig
   3-point: Open/Close move Pos [%] with 100 % per RunTime. Continuous: Pos follows Y [%] at the same
   speed. AtOpen/AtClosed are the end switches. P0: the start position. *)
VAR_INPUT Open : BOOL; Close : BOOL; Continuous : BOOL; Y : REAL; RunTime : TIME := T#120s; P0 : REAL; END_VAR
VAR_OUTPUT Pos : REAL; AtOpen : BOOL; AtClosed : BOOL; Moving : BOOL; END_VAR
VAR T : __Dt; Init : BOOL; Rate : REAL; Old : REAL; END_VAR
T();
IF NOT Init THEN Pos := LIMIT(0.0, P0, 100.0); Init := TRUE; END_IF;
Old := Pos;
Rate := 100.0 / MAX(TIME_TO_REAL(RunTime) * 0.001, 0.1) * T.Dt;
IF Continuous THEN
    Pos := LIMIT(Pos - Rate, LIMIT(0.0, Y, 100.0), Pos + Rate);
ELSIF Open AND NOT Close THEN
    Pos := MIN(Pos + Rate, 100.0);
ELSIF Close AND NOT Open THEN
    Pos := MAX(Pos - Rate, 0.0);
END_IF;
Moving := Pos <> Old;
AtOpen := Pos >= 99.5;
AtClosed := Pos <= 0.5;
END_FUNCTION_BLOCK
`,
	},
	{
		group: "Simulation · Pneumatik/Hydraulik",
		source: `
FUNCTION_BLOCK COMPRESSOR
(* Kompressor: Liefermenge, Leistung und Austrittstemperatur über dem Gegendruck
   The speed follows On * SpeedSet [%] with the time constant RunUp [s]. Delivery [Nl/min] =
   FreeAirDelivery * speed, falling linearly with the outlet pressure Pout [bar g] to 0 at MaxPressure
   (re-expansion of the clearance volume). Power: adiabatic compression of the delivered air divided by
   the isentropic Efficiency; Tout: the outlet temperature of that compression. *)
VAR_INPUT On : BOOL; SpeedSet : REAL := 100.0; Pout : REAL; FreeAirDelivery : REAL := 50.0; MaxPressure : REAL := 10.0;
    RunUp : REAL := 1.0; Tin : REAL := 20.0; Efficiency : REAL := 0.6; Pamb : REAL := 1.01325; END_VAR
VAR_OUTPUT Flow : REAL; (* [Nl/min] *) Power : REAL; (* [W] *) Tout : REAL; (* [°C] *) Speed : REAL; (* [%] *) Overpressure : BOOL; END_VAR
VAR T : __Dt; Target : REAL; Ratio : REAL; Rk : REAL; Eta : REAL; END_VAR
T();
IF On THEN Target := LIMIT(0.0, SpeedSet, 100.0); ELSE Target := 0.0; END_IF;
IF RunUp > 0.0 THEN Speed := Target + (Speed - Target) * EXP(-T.Dt / RunUp); ELSE Speed := Target; END_IF;
Overpressure := Pout >= MaxPressure;
Flow := MAX(FreeAirDelivery, 0.0) * Speed / 100.0 * LIMIT(0.0, 1.0 - Pout / MAX(MaxPressure, 0.01), 1.0);
Ratio := MAX((Pout + Pamb) / Pamb, 1.0);
Rk := EXPT(Ratio, 0.2857143) - 1.0;            (* (p2/p1)^((k-1)/k) - 1, k = 1.4 *)
Eta := LIMIT(0.05, Efficiency, 1.0);
Power := 3.5 * Pamb * 1.0E5 * Flow / 60000.0 * Rk / Eta;
IF Speed > 1.0 THEN
    Tout := (Tin + 273.15) * (1.0 + Rk / Eta) - 273.15;
ELSE
    Tout := Tin;
END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK TANK
(* Druckspeicher: Druck aus zu- und abströmender Luft (isotherm), Ventil mit Durchfluss nach ISO 6358
   InFlow [Nl/min] fills it (e.g. SIM.COMPRESSOR.Flow), Consumption [Nl/min] is taken at any pressure
   (less near 0 bar). The outlet valve (Valve 0..100 %, conductance ValveC [Nl/min per bar abs] at
   100 %) blows to the ambient: choked below the critical pressure ratio 0.5, subsonic above. The safety
   valve limits the pressure to ReliefPressure (Relief: blowing). P0: the start pressure; a rising edge of
   Reset sets it again. *)
VAR_INPUT InFlow : REAL; Consumption : REAL; Valve : REAL; ValveC : REAL := 100.0; Volume : REAL := 10.0; (* [l] *)
    ReliefPressure : REAL := 11.0; P0 : REAL; Reset : BOOL; Pamb : REAL := 1.01325; END_VAR
VAR_OUTPUT Pressure : REAL; (* [bar g] *) OutFlow : REAL; (* valve [Nl/min] *) Relief : BOOL; StoredAir : REAL; (* above ambient [Nl] *) END_VAR
VAR T : __Dt; R : R_TRIG; Init : BOOL; P : REAL; N : INT; I : INT; H : REAL; C : REAL; Q : REAL; Qc : REAL; Rr : REAL; Ks : REAL; END_VAR
T();
R(CLK := Reset);
IF NOT Init OR R.Q THEN P := Pamb + MAX(P0, 0.0); Init := TRUE; END_IF;
C := MAX(ValveC, 0.0) * LIMIT(0.0, Valve, 100.0) / 100.0;
Ks := 1.01325 / MAX(Volume, 0.001) / 60.0;      (* bar/s per Nl/min *)
N := __Steps(T.Dt, Ks * C * 2.0);
H := T.Dt / INT_TO_REAL(N);
FOR I := 1 TO N DO
    Q := 0.0;
    IF P > Pamb AND C > 0.0 THEN
        Rr := Pamb / P;
        IF Rr <= 0.5 THEN
            Q := C * P;
        ELSE
            Q := C * P * SQRT(MAX(0.0, 1.0 - ((Rr - 0.5) / 0.5) * ((Rr - 0.5) / 0.5)));
        END_IF;
    END_IF;
    Qc := MAX(Consumption, 0.0) * LIMIT(0.0, (P - Pamb) / 0.5, 1.0);
    P := P + H * Ks * (MAX(InFlow, 0.0) - Q - Qc);
    IF P < Pamb THEN P := Pamb; END_IF;
END_FOR;
OutFlow := Q;
Relief := P - Pamb > ReliefPressure;
IF Relief THEN P := Pamb + ReliefPressure; END_IF;
Pressure := P - Pamb;
StoredAir := Pressure / 1.01325 * Volume;
END_FUNCTION_BLOCK

FUNCTION_BLOCK VALVE
(* Stellventil: Volumenstrom aus Hub, Kvs-Wert und Differenzdruck (Wasser)
   Flow [m3/h] = Kv * SQRT(Dp [bar] / Density [kg/l]); Kv = Kvs * characteristic(Pos %): linear, or
   equal-percentage with the rangeability R (Kv = Kvs * R^(h - 1), closed below 1 %). *)
VAR_INPUT Pos : REAL; Kvs : REAL := 2.5; Dp : REAL := 0.5; EqualPercentage : BOOL := TRUE; R : REAL := 50.0; Density : REAL := 1.0; END_VAR
VAR_OUTPUT Flow : REAL; (* [m3/h] *) Kv : REAL; END_VAR
VAR Hh : REAL; END_VAR
Hh := LIMIT(0.0, Pos, 100.0) / 100.0;
IF Hh < 0.01 THEN
    Kv := 0.0;
ELSIF EqualPercentage THEN
    Kv := Kvs * EXPT(MAX(R, 1.1), Hh - 1.0);
ELSE
    Kv := Kvs * Hh;
END_IF;
IF Dp > 0.0 THEN Flow := Kv * SQRT(Dp / MAX(Density, 0.01)); ELSE Flow := 0.0; END_IF;
END_FUNCTION_BLOCK
`,
	},
	{
		group: "Simulation · Wärme",
		source: `
FUNCTION_BLOCK HEATER
(* Heizelement: Temperatur aus elektrischer Leistung, Wärmekapazität, Konvektion und Strahlung
   Power = RatedPower * Level % while On. The element (Mass [kg], SpecificHeat [J/(kg K)]) loses heat
   by convection (HA [W/K]) and radiation (Emissivity, Area [m2]) to the Ambient and QExt [W] to a
   coupled body (SIM.HEATTRANSFER.Q). Energy: the electrical energy [Wh]. Reset (edge): Temp := T0. *)
VAR_INPUT On : BOOL; Level : REAL := 100.0; RatedPower : REAL := 1000.0; Ambient : REAL := 20.0; Mass : REAL := 0.5; SpecificHeat : REAL := 500.0;
    HA : REAL := 2.0; Emissivity : REAL := 0.8; Area : REAL := 0.02; QExt : REAL; T0 : REAL := 20.0; Reset : BOOL; END_VAR
VAR_OUTPUT Temp : REAL; (* [°C] *) Power : REAL; (* [W] *) Loss : REAL; (* to the ambient [W] *) Energy : LREAL; (* [Wh] *) END_VAR
VAR T : __Dt; R : R_TRIG; Init : BOOL; Cap : REAL; Tk : REAL; Ta : REAL; Rad : REAL; N : INT; I : INT; H : REAL; Lambda : REAL; END_VAR
T();
R(CLK := Reset);
IF NOT Init OR R.Q THEN Temp := T0; Init := TRUE; END_IF;
IF On THEN Power := MAX(RatedPower, 0.0) * LIMIT(0.0, Level, 100.0) / 100.0; ELSE Power := 0.0; END_IF;
Cap := MAX(Mass * SpecificHeat, 0.001);
Ta := Ambient + 273.15;
Tk := MAX(Temp, Ambient) + 273.15;
Lambda := (MAX(HA, 0.0) + 4.0 * 5.670374E-8 * MAX(Emissivity, 0.0) * MAX(Area, 0.0) * Tk * Tk * Tk) / Cap;
N := __Steps(T.Dt, Lambda);
H := T.Dt / INT_TO_REAL(N);
FOR I := 1 TO N DO
    Tk := Temp + 273.15;
    Rad := 5.670374E-8 * MAX(Emissivity, 0.0) * MAX(Area, 0.0) * (Tk * Tk * Tk * Tk - Ta * Ta * Ta * Ta);
    Loss := MAX(HA, 0.0) * (Temp - Ambient) + Rad;
    Temp := Temp + H * (Power - Loss - QExt) / Cap;
END_FOR;
Energy := Energy + Power * T.Dt / 3600.0;
END_FUNCTION_BLOCK

FUNCTION_BLOCK THERMALMASS
(* Wärmekapazität: Temperatur eines Körpers (Raum, Wasserspeicher, Werkstück) aus Wärmeströmen
   Q1..Q3 [W] flow into the body (negative: out), Capacity [J/K] (water: 4190 per kg), UA [W/K] the loss
   to the Ambient. Exact solution over each step (stable for any cycle). Reset (edge): Temp := T0. *)
VAR_INPUT Q1 : REAL; Q2 : REAL; Q3 : REAL; Capacity : REAL := 100000.0; UA : REAL := 10.0; Ambient : REAL := 20.0; T0 : REAL := 20.0; Reset : BOOL; END_VAR
VAR_OUTPUT Temp : REAL; (* [°C] *) Loss : REAL; (* to the ambient [W] *) Stored : REAL; (* above T0 [kJ] *) END_VAR
VAR T : __Dt; R : R_TRIG; Init : BOOL; Q : REAL; Teq : REAL; Cap : REAL; END_VAR
T();
R(CLK := Reset);
IF NOT Init OR R.Q THEN Temp := T0; Init := TRUE; END_IF;
Cap := MAX(Capacity, 0.001);
Q := Q1 + Q2 + Q3;
IF UA > 0.0 THEN
    Teq := Ambient + Q / UA;
    Temp := Teq + (Temp - Teq) * EXP(-UA * T.Dt / Cap);
ELSE
    Temp := Temp + Q * T.Dt / Cap;
END_IF;
Loss := MAX(UA, 0.0) * (Temp - Ambient);
Stored := (Temp - T0) * Cap / 1000.0;
END_FUNCTION_BLOCK

FUNCTION_BLOCK HEATTRANSFER
(* Wärmeübergang: Wärmestrom zwischen zwei Körpern durch Leitung, Konvektion und Strahlung
   Q [W] from body 1 (T1) to body 2 (T2) = (UA + Conductivity * Area / Thickness + Alpha * Area) * (T1 - T2)
   + Emissivity * sigma * Area * (T1^4 - T2^4); the parts that are 0 drop out. Feed Q to body 1 as a loss
   (HEATER.QExt) and to body 2 as a gain (THERMALMASS.Q1). *)
VAR_INPUT T1 : REAL; T2 : REAL; UA : REAL; Conductivity : REAL; (* lambda [W/(m K)] *) Thickness : REAL; (* [m] *)
    Alpha : REAL; (* convection [W/(m2 K)] *) Area : REAL := 1.0; Emissivity : REAL; END_VAR
VAR_OUTPUT Q : REAL; (* [W] *) UAeff : REAL; (* [W/K] without radiation *) END_VAR
VAR A : REAL; B : REAL; END_VAR
UAeff := MAX(UA, 0.0) + MAX(Alpha, 0.0) * MAX(Area, 0.0);
IF Thickness > 0.0 THEN UAeff := UAeff + MAX(Conductivity, 0.0) * MAX(Area, 0.0) / Thickness; END_IF;
A := T1 + 273.15; B := T2 + 273.15;
Q := UAeff * (T1 - T2) + MAX(Emissivity, 0.0) * 5.670374E-8 * MAX(Area, 0.0) * (A * A * A * A - B * B * B * B);
END_FUNCTION_BLOCK

FUNCTION_BLOCK HEATEXCHANGER
(* Wärmeübertrager: Austrittstemperaturen zweier Stoffströme (epsilon-NTU), Gegen- oder Gleichstrom
   Hot side THotIn, MHot [kg/s], CpHot [J/(kg K)] (water 4190); cold side the same (air 1006); UA [W/K].
   Q [W] = Effectiveness * Cmin * (THotIn - TColdIn); the outlet temperatures follow with the exchanger's
   inertia Tau [s] (PT1). Without flow on a side its outlet keeps the exchanger's temperature. *)
VAR_INPUT THotIn : REAL := 70.0; MHot : REAL := 0.1; CpHot : REAL := 4190.0; TColdIn : REAL := 10.0; MCold : REAL := 0.5; CpCold : REAL := 1006.0;
    UA : REAL := 500.0; Counterflow : BOOL := TRUE; Tau : REAL := 10.0; END_VAR
VAR_OUTPUT THotOut : REAL; TColdOut : REAL; Q : REAL; (* [W] *) Effectiveness : REAL; END_VAR
VAR T : __Dt; Init : BOOL; Ch : REAL; Cc : REAL; Cmin : REAL; Cmax : REAL; Cr : REAL; Ntu : REAL; Ex : REAL; Th : REAL; Tc : REAL; F : REAL; END_VAR
T();
Ch := MAX(MHot, 0.0) * CpHot;
Cc := MAX(MCold, 0.0) * CpCold;
Cmin := MIN(Ch, Cc); Cmax := MAX(Ch, Cc);
IF Cmin <= 0.0 OR UA <= 0.0 THEN
    Effectiveness := 0.0; Q := 0.0;
    IF Ch > 0.0 THEN Th := THotIn; ELSE Th := THotOut; END_IF;
    IF Cc > 0.0 THEN Tc := TColdIn; ELSE Tc := TColdOut; END_IF;
ELSE
    Cr := Cmin / Cmax;
    Ntu := UA / Cmin;
    IF NOT Counterflow THEN
        Effectiveness := (1.0 - EXP(-Ntu * (1.0 + Cr))) / (1.0 + Cr);
    ELSIF Cr > 0.999 THEN
        Effectiveness := Ntu / (1.0 + Ntu);
    ELSE
        Ex := EXP(-Ntu * (1.0 - Cr));
        Effectiveness := (1.0 - Ex) / (1.0 - Cr * Ex);
    END_IF;
    Q := Effectiveness * Cmin * (THotIn - TColdIn);
    Th := THotIn - Q / Ch;
    Tc := TColdIn + Q / Cc;
END_IF;
IF NOT Init OR Tau <= 0.0 THEN
    THotOut := Th; TColdOut := Tc; Init := TRUE;
ELSE
    F := 1.0 - EXP(-T.Dt / Tau);
    THotOut := THotOut + (Th - THotOut) * F;
    TColdOut := TColdOut + (Tc - TColdOut) * F;
END_IF;
END_FUNCTION_BLOCK
`,
	},
	{
		group: "Simulation · Regelstrecken",
		source: `
FUNCTION_BLOCK PT1
(* PT1-Glied: Verzögerung erster Ordnung (Out = K * In, Zeitkonstante Tau [s]) *)
VAR_INPUT In : REAL; K : REAL := 1.0; Tau : REAL := 10.0; Y0 : REAL; END_VAR
VAR_OUTPUT Out : REAL; END_VAR
VAR T : __Dt; Init : BOOL; END_VAR
T();
IF NOT Init THEN Out := Y0; Init := TRUE; END_IF;
IF Tau > 0.0 THEN Out := K * In + (Out - K * In) * EXP(-T.Dt / Tau); ELSE Out := K * In; END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK DEADTIME
(* Totzeit: der Eingang erscheint Delay später am Ausgang (Transport in Rohren, Förderbändern)
   100 samples over Delay (resolution Delay / 100); Out starts with Y0. *)
VAR_INPUT In : REAL; Delay : TIME := T#10s; Y0 : REAL; END_VAR
VAR_OUTPUT Out : REAL; END_VAR
VAR T : __Dt; Buf : ARRAY[0..99] OF REAL; Head : INT; Acc : TIME; Ts : TIME; Init : BOOL; I : INT; END_VAR
T();
IF NOT Init THEN
    FOR I := 0 TO 99 DO Buf[I] := Y0; END_FOR;
    Out := Y0; Init := TRUE;
END_IF;
Ts := Delay / 100;
IF Ts <= T#0s THEN
    Out := In;
ELSE
    Acc := Acc + T.DtT;
    WHILE Acc >= Ts DO
        Acc := Acc - Ts;
        Out := Buf[Head];
        Buf[Head] := In;
        Head := (Head + 1) MOD 100;
    END_WHILE;
END_IF;
END_FUNCTION_BLOCK
`,
	},
];

/** The library's source: the namespace with the helpers and every section. */
export function simLibSource(): string {
	return `NAMESPACE ${SIM_NAMESPACE}\n${HELPERS}\n${SECTIONS.map((s) => s.source).join("\n")}\nEND_NAMESPACE\n`;
}

export function simSections(): readonly Section[] {
	return SECTIONS;
}
