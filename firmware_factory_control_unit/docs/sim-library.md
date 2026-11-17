# Bibliothek SIM – Realsimulatoren

Funktionsbausteine, die das dynamische Verhalten realer Betriebsmittel nachbilden: Sie bilden physikalische
Größen an den Eingängen auf physikalische Größen an den Ausgängen ab. Ein Programm kann damit seine
Anlage selbst simulieren, im Browser-Simulator **und** auf der FCU (virtuelle Inbetriebnahme, Unterricht
ohne Hardware). Quelltext `web_packages/plc/src/ide/sim-lib.ts`, Tests `web_packages/plc/src/ide/sim-lib.test.ts` (gegen
analytische Lösungen und Energie-/Massenbilanzen).

Namensraum `SIM` (`m : SIM.MOTOR;` oder `USING SIM;`), in der FBD-Palette unter „Simulation · …“.

## Gemeinsame Regeln

- **Zeit:** wie bei der VDI3814-Bibliothek ([vdi3814-library.md](vdi3814-library.md#gemeinsame-regeln-gelten-für-alle-bausteine)):
  `dt` aus der SPS-Zeit, erster Aufruf `dt = 0`, höchstens 1 s je Aufruf. Nach einem Haltepunkt läuft die
  Simulation also mit höchstens 1 s weiter, ohne Sprung.
- **Numerik:** Jeder Baustein bleibt bei jeder Zykluszeit stabil. Lineare Anteile werden exakt gelöst
  (Exponentialfunktion: PT1, Wärmekapazität, Kompressordrehzahl, Wärmeübertrager-Trägheit) oder
  implizit (Motor). Nichtlineare Anteile (Ventil, Strahlung) werden in bis zu 200 Teilschritte zerlegt, so
  dass jeder Teilschritt höchstens 20 % der lokalen Zeitkonstante ist (getestet: Motor mit 1 µs-Trägheit
  bei 1 s Zyklus, Kessel mit 10 ml und weit offenem Ventil). Wird mehr gebraucht, bleibt das Ergebnis
  stabil, aber ungenauer.
- **Kopplung:** Gekoppelte Bausteine (Motor → Spindel, Heizelement → Wärmeübergang → Wärmekapazität)
  tauschen ihre Werte einmal pro Zyklus aus, explizit, mit einem Zyklus Versatz. Das ist genau, solange
  die Zeitkonstanten der Körper groß gegen den Zyklus sind (`C/UA ≫ Zyklus`). Die Energiebilanz geht
  dabei nicht verloren (getestet: Heizelement → Übergang → Masse, 120 kJ zugeführt = 120 kJ gespeichert).
- **Einheiten:** SI, sofern nicht anders angegeben; Temperaturen in °C, Drücke in bar Überdruck, Luftmengen
  in Normlitern [Nl] (1,01325 bar, 0 °C), Positionen in mm bzw. %.
- **Startzustand:** über die Eingänge `P0`, `T0`, `Y0`; einige Bausteine setzen ihn mit einer Flanke an
  `Reset`/`Set` neu. Ein Warmstart der SPS beginnt die Simulation von vorn.

## Antriebe

### SIM.MOTOR – Motor

Eingänge: `U` [%] = 100, `On`, `LoadTorque` [Nm], `Brake`, `NoLoadSpeed` [1/min] = 3000,
`StallTorque` [Nm] = 2, `StallCurrent` [A] = 10, `NoLoadCurrent` [A] = 0,3, `Inertia` [kg·m²] = 0,0005,
`Friction` [Nm je 1000 1/min] = 0,01.
Ausgänge: `Speed` [1/min], `Torque` [Nm], `Current` [A], `Power` [W], `Revs` (Umdrehungen, LREAL).

Modell: Gleichstrommotor mit linearer Drehmomentkennlinie. Für einen Asynchronmotor am Frequenzumrichter
gilt dasselbe im Arbeitsbereich (Schlupfgerade).

- Motormoment `M = StallTorque·(u − n/NoLoadSpeed)` mit `u = LIMIT(−100, U, 100)/100`. Bei `On = FALSE`
  ist `M = 0` und der Motor trudelt aus. Bei `On` mit `U = 0` bremst er generatorisch (Kurzschlussbremse
  des Gleichstrommotors).
- Bewegungsgleichung `J·dω/dt = M − M_Last − M_Reibung`, implizit gelöst. Mechanische Zeitkonstante
  `τ = J·ω0/StallTorque` (getestet: 63 % nach τ).
- **Last** `LoadTorque` ist passiv (Reibung, Prozess): Sie wirkt gegen die Bewegungsrichtung und hält im
  Stillstand, solange `|M| ≤ LoadTorque` (Haftung, getestet: 3 Nm Last gegen 2 Nm Kippmoment → steht,
  Strom = Kurzschlussstrom). Ein Nulldurchgang der Drehzahl durch die Last endet im Stillstand und schwingt
  nicht durch. **Eine ziehende Last** (Hubwerk, Gewicht) ist damit nicht darstellbar.
- Stationär ohne Reibung: `n = NoLoadSpeed·(u − LoadTorque/StallTorque)` (getestet: halbe Last → halbe
  Drehzahl).
- `Current = M/StallTorque·StallCurrent`, plus `NoLoadCurrent` (mit Vorzeichen) bei Drehung, 0 bei
  `On = FALSE`.
- `Power = M·ω` (Wellenleistung des Motormoments; negativ = generatorisch).
- `Brake` hält die Welle (Drehzahl 0). Das Motormoment wird weiter ausgegeben (Strom fließt).

### SIM.SPINDLE – Drehspindel (Integrator Drehbewegung → Weg)

Eingänge: `Speed` [1/min], `Ratio` = 1 (Motor : Spindel), `Pitch` [mm/U] = 4, `MinPos` = 0,
`MaxPos` = 300 [mm], `LimitWindow` [mm] = 1, `Set`, `SetPos`, `P0`.
Ausgänge: `Pos` [mm], `Velocity` [mm/s], `AtMin`, `AtMax`, `Blocked`, `Revs`.

- `Velocity = Speed/|Ratio|/60·Pitch`, negatives `Ratio` kehrt die Richtung um; `Pos` integriert sie
  (intern LREAL, keine Drift über lange Wege; getestet).
- **Endanschläge:** `Pos` bleibt in `MinPos..MaxPos`. Am Anschlag mit Bewegung in Richtung Anschlag ist
  `Blocked = TRUE`, `Velocity = 0`, und die Spindelumdrehungen zählen nicht (getestet). Rückwärts geht
  es sofort frei.
- Endschalter `AtMin`/`AtMax`: `LimitWindow` vor dem Anschlag (Endschalter vor dem mechanischen
  Anschlag, für die Referenzfahrt).
- **Rückwirkung:** Die Spindel kennt keine Kräfte. Ein Motor am Anschlag dreht im Modell weiter, solange
  man `Blocked` nicht zurückführt, z. B. `m(Brake := sp.Blocked …)`, oder realistischer als Last:
  `LoadTorque := SEL(sp.Blocked, 0.1, 10.0)`.
- Flanke `Set` → `Pos := SetPos` (Referenzpunkt setzen); `P0`: Startposition beim ersten Aufruf.

### SIM.ACTUATOR – Stellantrieb

Eingänge: `Open`, `Close`, `Continuous`, `Y` [%], `RunTime` = 120 s, `P0`.
Ausgänge: `Pos` [%], `AtOpen`, `AtClosed`, `Moving`.

Laufzeitantrieb (100 % in `RunTime`, Mindestlaufzeit 0,1 s). 3-Punkt: `Open`/`Close` fahren; **beide
gleichzeitig: Stillstand** (getestet). Stetig (`Continuous`): fährt mit derselben Geschwindigkeit auf `Y`.
Endschalter bei ≥ 99,5 % bzw. ≤ 0,5 %. Passt zu `VDI3814.OUT3P`/`OUT3PVAR` (deren `Pos` ist das Modell im
Regler, `SIM.ACTUATOR.Pos` die „wirkliche“ Stellung).

## Pneumatik/Hydraulik

### SIM.COMPRESSOR – Kompressor

Eingänge: `On`, `SpeedSet` [%] = 100, `Pout` [bar Ü], `FreeAirDelivery` [Nl/min] = 50, `MaxPressure`
[bar Ü] = 10, `RunUp` [s] = 1, `Tin` [°C] = 20, `Efficiency` = 0,6, `Pamb` [bar abs] = 1,01325.
Ausgänge: `Flow` [Nl/min], `Power` [W], `Tout` [°C], `Speed` [%], `Overpressure`.

- Drehzahl: PT1 mit `RunUp` auf `On·SpeedSet` (Hoch- und Auslauf, getestet).
- Liefermenge: `Flow = FAD·Speed/100·(1 − Pout/MaxPressure)`, begrenzt ≥ 0. Der lineare Abfall bildet die
  Rückexpansion des Schadraums nach; bei `MaxPressure` fördert der Kompressor nichts mehr
  (`Overpressure`, getestet).
- Leistung: adiabate Verdichtung `P = κ/(κ−1)·p1·V̇·((p2/p1)^((κ−1)/κ) − 1)/η`, κ = 1,4 (getestet gegen die
  Formel). Ohne Förderung ist `P = 0`; Leerlaufverluste sind nicht modelliert.
- `Tout = T1·(1 + ((p2/p1)^0,2857 − 1)/η)`, ohne Kühlung, also die Verdichtungsendtemperatur. Ab
  `Speed ≤ 1 %` gilt `Tout = Tin` (keine Abkühlkurve).
- Ein Gegendruck unter 0 bar Ü (Vakuum) erhöht die Förderung nicht über `FAD`.

### SIM.TANK – Druckspeicher

Eingänge: `InFlow` [Nl/min], `Consumption` [Nl/min], `Valve` [%], `ValveC` [Nl/(min·bar abs)] = 100,
`Volume` [l] = 10, `ReliefPressure` [bar Ü] = 11, `P0` [bar Ü], `Reset`, `Pamb` = 1,01325.
Ausgänge: `Pressure` [bar Ü], `OutFlow` [Nl/min], `Relief`, `StoredAir` [Nl über Umgebung].

- Isotherme Massenbilanz: `dp/dt = 1,01325/V·(Q_zu − Q_Ventil − Q_Verbraucher)/60` (getestet: 60 Nl/min
  für 10 s in 10 l → +1,013 bar).
- Ventil nach ISO 6358 mit dem Leitwert `C = ValveC·Valve/100` und dem kritischen Druckverhältnis
  b = 0,5. Überkritisch (`pamb/p ≤ 0,5`): `Q = C·p` (Schallgeschwindigkeit, getestet). Unterkritisch:
  `Q = C·p·√(1 − ((r − b)/(1 − b))²)`. Bei `p ≤ pamb` fließt nichts (kein Rückfluss).
- `Consumption` ist ein Verbraucher mit fester Entnahme, der unter 0,5 bar Ü linear auf 0 zurückgeht.
  Der Druck wird so nie negativ.
- Sicherheitsventil: ideal, begrenzt hart auf `ReliefPressure`, `Relief` meldet das Abblasen (getestet).
- Temperaturänderungen beim Füllen/Entleeren sind nicht modelliert (isotherm, also für langsame Vorgänge).

### SIM.VALVE – Stellventil (Wasser)

Eingänge: `Pos` [%], `Kvs` [m³/h] = 2,5, `Dp` [bar] = 0,5, `EqualPercentage` = TRUE, `R` = 50,
`Density` [kg/l] = 1.
Ausgänge: `Flow` [m³/h], `Kv`.

`Flow = Kv·√(Dp/Density)`; `Kv = Kvs·h` (linear) oder `Kvs·R^(h−1)` (gleichprozentig), `h = Pos/100`.
Unter 1 % Hub ist das Ventil dicht (der gleichprozentige Kegel würde sonst bei 0 % noch Kvs/R
durchlassen). `Dp ≤ 0` → kein Fluss (kein Rückfluss). Kein Kavitations-/Drosselgrenzmodell.

## Wärme

### SIM.HEATER – Heizelement

Eingänge: `On`, `Level` [%] = 100, `RatedPower` [W] = 1000, `Ambient` [°C] = 20, `Mass` [kg] = 0,5,
`SpecificHeat` [J/(kg·K)] = 500, `HA` [W/K] = 2, `Emissivity` = 0,8, `Area` [m²] = 0,02, `QExt` [W],
`T0` = 20, `Reset`.
Ausgänge: `Temp` [°C], `Power` [W], `Loss` [W], `Energy` [Wh].

`m·c·dT/dt = P − HA·(T − Ta) − ε·σ·A·(T⁴ − Ta⁴) − QExt` (getestet: stationär ohne Strahlung 20 + 100 W /
2 W/K = 70 °C, Zeitkonstante m·c/HA; mit Strahlung die Bilanz im Beharrungszustand). Kein Schmelzen,
keine Temperaturgrenze, keine Widerstandsänderung mit der Temperatur. `Energy` zählt die elektrische
Energie.

### SIM.THERMALMASS – Wärmekapazität

Eingänge: `Q1`, `Q2`, `Q3` [W, positiv = hinein], `Capacity` [J/K] = 100 000, `UA` [W/K] = 10,
`Ambient` = 20, `T0` = 20, `Reset`.
Ausgänge: `Temp` [°C], `Loss` [W], `Stored` [kJ über T0].

Exakte Lösung über jeden Schritt: `T → Ta + Q/UA` mit `τ = C/UA` (getestet bei 1 s Zyklus); `UA = 0`:
reine Integration (getestet). Für Raum, Speicher, Werkstück; die Wärmeströme kommen aus `HEATTRANSFER`,
`HEATER` (dessen `Power`) oder Rechnungen wie `m_punkt·cp·(T_vor − T_rück)`.

### SIM.HEATTRANSFER – Wärmeübergang

Eingänge: `T1`, `T2` [°C], `UA` [W/K], `Conductivity` λ [W/(m·K)], `Thickness` [m], `Alpha`
[W/(m²·K)], `Area` [m²] = 1, `Emissivity`.
Ausgänge: `Q` [W, von 1 nach 2], `UAeff` [W/K].

`Q = (UA + λ·A/d + α·A)·(T1 − T2) + ε·σ·A·(T1⁴ − T2⁴)`; Anteile mit 0 entfallen, `Thickness ≤ 0` schaltet
die Leitung ab (getestet). Zustandslos: `Q` als Verlust an Körper 1 (`HEATER.QExt` oder `−Q` an eine
`THERMALMASS`) und als Gewinn an Körper 2 geben. Die Strahlung ist die zwischen zwei parallelen Flächen
mit Sichtfaktor 1 und dem effektiven Emissionsgrad `Emissivity`.

### SIM.HEATEXCHANGER – Wärmeübertrager

Eingänge: `THotIn` = 70, `MHot` [kg/s] = 0,1, `CpHot` = 4190, `TColdIn` = 10, `MCold` = 0,5,
`CpCold` = 1006, `UA` [W/K] = 500, `Counterflow` = TRUE, `Tau` [s] = 10.
Ausgänge: `THotOut`, `TColdOut`, `Q` [W], `Effectiveness`.

ε-NTU-Verfahren: Gegenstrom `ε = (1 − e^(−NTU(1−Cr)))/(1 − Cr·e^(−NTU(1−Cr)))` (für `Cr → 1`:
`NTU/(1+NTU)`), Gleichstrom `ε = (1 − e^(−NTU(1+Cr)))/(1 + Cr)` (getestet: Cr = 1, NTU → ∞ → 0,5).
Energiebilanz beider Seiten erfüllt (getestet). Die Austrittstemperaturen folgen dem stationären Wert mit
PT1 `Tau` (Speicherwirkung des Übertragers). Ohne Durchfluss auf einer Seite (oder `UA ≤ 0`): `Q = 0`,
eine durchströmte Seite gibt ihre Eintrittstemperatur aus, eine stehende behält ihren letzten Wert.

## Regelstrecken

### SIM.PT1

Eingänge: `In`, `K` = 1, `Tau` [s] = 10, `Y0`. Ausgang: `Out`.
Exakte Lösung (getestet); `Tau ≤ 0`: proportional ohne Verzögerung. Startwert `Y0`.

### SIM.DEADTIME – Totzeit

Eingänge: `In`, `Delay` = 10 s, `Y0`. Ausgang: `Out`.
100 Abtastwerte über `Delay`, Auflösung `Delay/100`; `Out` ist der vor `Delay` abgetastete Wert, am
Anfang `Y0` (getestet). Ist die Abtastzeit kürzer als der Zyklus, werden pro Zyklus mehrere Werte
geschoben, alle mit dem aktuellen Eingang (Treppenform). `Delay` während des Betriebs zu ändern verzerrt
die Werte im Puffer, verliert aber keine. `Delay = 0`: unverzögert.

## Beispiel: Druckluftstation

```iecst
USING SIM;
PROGRAM PLC_PRG
VAR
    Kompressor : COMPRESSOR;
    Kessel : TANK;
    Druckregler : VDI3814.LIMITMON;     (* Zweipunkt 6..8 bar *)
    Verbraucher : REAL := 15.0;         (* Nl/min, von der Bedienoberfläche *)
END_VAR
Druckregler(In := Kessel.Pressure, High := 8.0, Low := 6.0, Hyst := 0.0);
IF Druckregler.QLow THEN Kompressor.On := TRUE; ELSIF Druckregler.QHigh THEN Kompressor.On := FALSE; END_IF;
Kompressor(Pout := Kessel.Pressure);
Kessel(InFlow := Kompressor.Flow, Consumption := Verbraucher, Volume := 20.0);
END_PROGRAM
```
