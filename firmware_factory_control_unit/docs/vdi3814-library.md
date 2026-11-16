# Bibliothek VDI3814 – GA-Funktionen als Funktionsbausteine

Die Bibliothek setzt die GA-Funktionen der **VDI 3814 Blatt 4.3:2022** (Spalten der GA-Funktionsliste)
als IEC-61131-3-Bausteine für die SPS der FCU um. Quelltext: `web/src/plc/ide/vdi3814-lib.ts` (ST),
Tests: `web/src/plc/ide/vdi3814-lib.test.ts` (jede hier genannte Grenzfall-Aussage mit „getestet“ ist dort
abgesichert).

Die Richtlinie beschreibt *was* eine Funktion leistet, nicht *wie*. Dieses Dokument legt die konkrete
Funktionsweise fest, die diese Bibliothek umsetzt. Wo die Auslegung eine Wahl war, ist sie unter
**Auslegung** begründet.

| VDI-3814-Gruppe | Bausteine |
|---|---|
| 1.1 E/A physikalisch | `AI`, `BI`, `AO`, `BO` |
| 1.2 E/A Werte | `AV`, `BV`, `MV` (Mehrstufenwert) |
| 1.3 E/A komplex | `SCH` (Zeitplan), `CAL` (Kalender), `NC` (Alarm-/Ereignismeldung), `LOG` (Datenaufzeichnung), dazu `CLOCK` |
| 2.1 Logik | Zeiten: Standard `TP`/`TON`/`TOF` und `TONOF`; Arithmetik: Standardoperatoren und `SCALE`, `MEAN4` |
| 2.2 Überwachen/Steuern | `LIMITMON`, `RUNHOURS`, `EVENTCOUNT`, `CMDMON`, `DEVSTATUS`, `MOTOR`, `ANTISEIZE`, `ASWITCH`, `SEQUENCE`, `FROST`, `SAFETY`, `PRIORITY` |
| 2.3 Regelung | `PID`, `SPCURVE`, `OUTCONT`, `OUT2P`, `OUT3P`, `OUT3PVAR`, `PWM`, `LIMITSP`, `PARAMSET` |
| 2.4 Optimierung | `ENERGYLEVEL`, `HEATRECOVERY`, `HXSETPOINT`, `OPTSTART`, `NIGHTCOOL` |
| 2.5 Beleuchtung | `LIGHT`, `STAIRLIGHT`, `DAYLIGHT`, `CONSTLIGHT` |
| 2.6 Sonnenschutz | `SUNBLIND`, `SUNDUSK`, `THERMOAUTO`, `WEATHERPROT`, `SUNPOS` |
| 3 Bedienen/Anzeigen/Management | die Bedienoberfläche (HMI) der SPS-Seite, siehe [hmi.md](hmi.md) |

Nicht umgesetzt: 1.3 „Sonstige komplexe Objekte (KO)“ und 2.7 „Sonstige Anwendungsfunktionen“. Beides
sind Sammelspalten ohne festen Inhalt.

## Verwendung

- Die Bausteine liegen im Namensraum `VDI3814`: in ST `regler : VDI3814.PID;` oder nach `USING VDI3814;`
  einfach `regler : PID;`. Im FBD-Editor stehen sie in der Palette unter „VDI 3814 · …“, der Tooltip zeigt
  den Titel.
- Die Bibliothek wird wie die FCU-Bibliothek mit jedem Projekt übersetzt; ins Abbild kommen nur die
  benutzten Bausteine.
- Aufruf in ST nur formal (`regler(SP := 21.0, PV := t);`). Nicht übergebene Eingänge behalten ihren
  letzten Wert, beim ersten Aufruf also den Vorgabewert. Im FBD gilt dasselbe für nicht verbundene
  Eingänge.

## Gemeinsame Regeln (gelten für alle Bausteine)

**Zeitbasis.** Jeder zeitabhängige Baustein misst den Zeitschritt `dt` selbst: die Startzeit des
aktuellen Zyklus (`__PLC_TIME`) minus die Startzeit beim letzten Aufruf *dieser Instanz*. Daraus folgt:

- Der **erste Aufruf** einer Instanz hat `dt = 0`. Zeiten, Integrale und Zähler laufen erst ab dem
  zweiten Aufruf (getestet: `RUNHOURS` zählt nach 4 Aufrufen à 1 s nur 3 s).
- `dt` ist auf **höchstens 1 s** begrenzt. Nach einem Halt am Haltepunkt, einem STOP oder einer
  ausgelassenen Bearbeitung springen Integrale, Rampen und Zeitzähler also nicht. Wird eine Instanz
  seltener als einmal pro Sekunde aufgerufen (Task-Intervall > 1 s, bedingter Aufruf), laufen ihre
  Zeiten entsprechend langsamer.
- Rufe eine Instanz **genau einmal pro Zyklus** auf. Ein zweiter Aufruf im selben Zyklus sieht `dt = 0`,
  bewirkt also keine Zeitänderung, wertet aber die Logik erneut aus.
- Zeitzähler vom Typ TIME (z. B. Mindestzeiten) sättigen bei 24 Tagen, statt überzulaufen.

**Flanken.** „Flanke“ heißt immer steigende Flanke, erkannt zwischen zwei Aufrufen der Instanz. Liegt
ein Flankeneingang schon beim **ersten Aufruf** auf TRUE, zählt das als Flanke (Verhalten des
Standard-`R_TRIG`).

**Werte schreiben.** Die Ausgänge `Value` von `AV`, `BV` und `MV` sind auch direkt beschreibbar
(Bedienoberfläche, Watchliste). Der Baustein übernimmt den geschriebenen Wert im nächsten Zyklus und hält
dabei seine Grenzen ein.

**Grenzen ohne Wert.** `-1.0E30`/`1.0E30` als Vorgabe heißt „keine Grenze“.

**Zähler** (UDINT) laufen nach 4 294 967 295 auf 0 über (Ganzzahlarithmetik der SPS).

**Kein Laufzeitfehler aus der Bibliothek.** Division durch null und Wurzeln/Logarithmen negativer Zahlen
sind in allen Bausteinen abgefangen (siehe die jeweiligen Grenzfälle). Unsinnige Parameter führen zu
einem definierten Ersatzverhalten, nicht zum ERROR-Zustand der SPS. Ausnahme sind die Ganzzahl-Ausgänge
von `CLOCK` bei Datumswerten außerhalb von INT (Jahr > 32767).

---

## 1.1 Ein-/Ausgabe physikalisch

### VDI3814.AI – Analoge Eingabe

Eingänge: `Raw` (Rohwert), `RawLo` = 0, `RawHi` = 4095, `EngLo` = 0, `EngHi` = 100, `Offset` = 0,
`Filter` [s] = 0, `FaultLo` = −1E30, `FaultHi` = 1E30, `OutOfService`, `Manual`.
Ausgänge: `PV` (Messwert), `Fault`.

1. `Fault := Raw < FaultLo OR Raw > FaultHi` (Drahtbruch/Kurzschluss als Rohwert außerhalb des gültigen
   Bereichs; die Grenzen selbst sind gültig).
2. Skalierung: `X = EngLo + (Raw − RawLo)·(EngHi − EngLo)/(RawHi − RawLo) + Offset`, linear und **ohne
   Begrenzung** auf EngLo..EngHi. Ein Rohwert über RawHi liefert also mehr als EngHi.
3. Filter PT1 mit Zeitkonstante `Filter`: `F += (X − F)·(1 − e^(−dt/Filter))`. Das ist die exakte Lösung,
   stabil für jedes Verhältnis von Zyklus zu Filterzeit.
4. `PV = Manual` bei `OutOfService`, sonst `F`.

Grenzfälle:
- `RawLo = RawHi`: `X = EngLo + Offset`, keine Division (getestet).
- Bei `Fault` friert das Filter ein und `PV` **hält den letzten guten Wert** (getestet). Ist der Rohwert
  schon beim Start ungültig, bleibt `PV` = 0, bis ein gültiger Wert kommt (getestet). Nach dem Ende der
  Störung läuft das Filter vom letzten guten Wert weiter.
- `Filter ≤ 0`: kein Filter. Beim ersten gültigen Wert springt das Filter auf den Messwert, statt von 0
  hochzulaufen.
- `OutOfService` wirkt auch während einer Störung; `Fault` wird weiter gemeldet.

### VDI3814.BI – Binäre Eingabe

Eingänge: `In`, `Invert`, `Debounce` (TIME), `OutOfService`, `Manual`, `ResetCount`.
Ausgänge: `PV`, `Changes` (UDINT), `ActiveTime` [h] (LREAL).

- Der Kandidat `In XOR Invert` wird übernommen, wenn er ununterbrochen `Debounce` lang anstand. Jede
  Änderung startet die Wartezeit neu. Kürzere Störimpulse fallen in beide Richtungen weg (getestet).
  `Debounce = 0`: sofort.
- `PV = Manual` bei `OutOfService`, sonst der entprellte Wert. Zähler und Zeit laufen mit dem
  ausgegebenen `PV`, also auch mit dem Handwert.
- `Changes` zählt die Wechsel FALSE→TRUE von `PV`; `ActiveTime` summiert die Zeit mit `PV = TRUE`.
  `ResetCount` (Pegel, nicht Flanke) setzt beide auf 0 und hält sie dort, solange es TRUE ist.

**Auslegung:** Entprellt wird nach Polarität. Ein Wechsel von `Invert` wirkt deshalb erst nach
`Debounce`.

### VDI3814.AO – Analoge Ausgabe

Eingänge: `Auto`, `ManualMode`, `Manual`, `Min` = 0, `Max` = 100, `Ramp` [Einheit/s] = 0, `EngLo` = 0,
`EngHi` = 100, `OutLo` = 0, `OutHi` = 10.
Ausgänge: `PV` (Stellwert in Ingenieurseinheiten), `Out` (Ausgabesignal).

- Ziel = `Manual` bei `ManualMode`, sonst `Auto`, begrenzt mit `LIMIT(Min, Ziel, Max)`.
- `PV` läuft mit höchstens `Ramp`·dt pro Zyklus auf das Ziel zu; `Ramp ≤ 0`: sofort.
- `Out = OutLo + (PV − EngLo)·(OutHi − OutLo)/(EngHi − EngLo)` (z. B. 0..100 % → 0..10 V oder 2..10 V).

Grenzfälle:
- Der **erste Aufruf** übernimmt das Ziel ohne Rampe (getestet). Ein Neustart fährt den Ausgang also
  nicht von 0 hoch.
- `EngLo = EngHi`: `Out = OutLo`.
- `Min > Max`: `LIMIT` liefert dann `Max` (Standardverhalten von LIMIT mit MN > MX: MX).
- Auch der Handwert wird begrenzt und folgt der Rampe. Hand/Auto-Umschaltung ist also stoßfrei, wenn
  `Ramp > 0`.

### VDI3814.BO – Binäre Ausgabe

Eingänge: `Auto`, `ManualMode`, `Manual`, `MinOn`, `MinOff` (TIME), `Invert`.
Ausgänge: `Q` (an die Klemme), `PV` (logischer Zustand), `Switches` (UDINT).

- Gewünscht = `Manual` bei `ManualMode`, sonst `Auto`.
- Ein Wechsel ein→aus erst, wenn `PV` mindestens `MinOn` ein war; aus→ein erst nach mindestens `MinOff`
  aus (getestet). Ein Wunsch, der vor Ablauf wieder zurückgenommen wird, bewirkt nichts.
- `Q = PV XOR Invert`; `Switches` zählt jeden Wechsel von `PV`.

Grenzfälle:
- Das **erste Einschalten** nach dem Start wartet nicht auf `MinOff` (getestet). Die Ausschaltzeit vor
  dem Start ist unbekannt und wird als abgelaufen angenommen.
- **Auslegung:** Die Mindestzeiten gelten auch im Handbetrieb. Sie schützen das Gerät (Verdichter,
  Brenner) und sind kein Komfortmerkmal. Eine Abschaltung, die nicht auf `MinOn` warten darf
  (Sicherheitskette, siehe `SAFETY`), verknüpft man hinter dem Baustein: `Klemme := bo.Q AND safety.Release`.

## 1.2 Ein-/Ausgabe Werte

### VDI3814.AV – Analogwert

Eingänge: `In`, `Set`, `Reset`, `Default`, `Min` = −1E30, `Max` = 1E30.
Ausgänge: `Value`, `Changed`.

- Erster Aufruf: `Value := Default`.
- Flanke `Reset` → `Default`; sonst Flanke `Set` → `In`. Reset hat Vorrang.
- `Value` wird jeden Zyklus auf `Min..Max` begrenzt, **auch direkt geschriebene Werte** (getestet:
  geschriebene 99 bei Max 30 → 30).
- `Changed` ist für einen Zyklus TRUE, wenn `Value` am Ende des Aufrufs anders ist als am Ende des
  vorigen: durch Set, Reset oder einen von außen geschriebenen Wert, der im nächsten Zyklus gemeldet wird
  (getestet). So lassen sich Bedieneingriffe protokollieren (z. B. in eine `NC` mit Klasse 7 Handeingriff).

### VDI3814.BV – Binärwert

Eingänge: `Set`, `Reset`, `Toggle`, `Default`. Ausgang: `Value`.
Erster Aufruf `Value := Default`; Flanken mit Vorrang Reset > Set > Toggle (getestet: alle drei
gleichzeitig → FALSE). `Value` ist direkt beschreibbar.

### VDI3814.MV – Mehrstufenwert

Eingänge: `States` = 3, `In`, `Set`, `Up`, `Down`, `Default` = 1.
Ausgänge: `Value`, `Q1`..`Q4`.

- Erster Aufruf `Value := Default`; Flanken: `Set` → `In`, sonst `Up` +1, sonst `Down` −1.
- Danach immer `Value := LIMIT(1, Value, MAX(States, 1))`. Es gibt also kein Umlaufen von der letzten
  Stufe zur ersten (getestet), und `States ≤ 0` wirkt wie 1 (getestet).
- `Qn` ist TRUE für `Value = n`; über 4 Stufen gibt es nur `Value`.

## 1.3 Ein-/Ausgabe komplex

### VDI3814.CLOCK – Uhr (Hilfsbaustein)

Die SPS hat noch keine Echtzeituhr (DATE/TOD kommen mit P7). `CLOCK` führt eine Software-Uhr, die der
Bediener stellt; `SCH`, `CAL`, `OPTSTART` und `SUNPOS` brauchen ihre Ausgänge.

Eingänge: `Set`, `Year` = 2026, `Month` = 1, `Day` = 1, `Hour`, `Minute`, `Second`, `Speed` = 1.0.
Ausgänge: `TimeOfDay` (TIME seit Mitternacht), `Weekday` (1 = Montag … 7 = Sonntag), `DayOfYear`,
`Y`, `Mo`, `D`, `H`, `Mi`, `S`.

- Der erste Aufruf und jede Flanke an `Set` übernehmen Datum und Uhrzeit von den Eingängen.
- Danach läuft die Uhr mit `dt·Speed` (`Speed` 60 = eine Minute pro Sekunde, für Tests und
  Vorführungen; negative Werte zählen als 0).
- Proleptischer gregorianischer Kalender mit Schaltjahren (getestet: 31.12.2024 23:59:59 + 1 s = Mittwoch,
  1.1.2025, Tag 1; 28.2.2028 + 12 h → 29.2.).

Grenzfälle:
- Ungültige Eingaben werden **nicht** geprüft, sondern nach Kalenderarithmetik normalisiert: 32. Januar =
  1. Februar, Monat 13 = Januar des Folgejahres, Stunde 25 = 1 Uhr des Folgetages.
- Keine Sommer-/Winterzeit: Die Uhr zeigt die eingestellte Zeit. Bei der Umstellung stellt man neu.
- Nach einem Spannungsausfall oder Warmstart beginnt die Uhr wieder bei den Eingangswerten, da RETAIN noch
  nicht unterstützt wird.

### VDI3814.SCH – Zeitplan

Eingänge: `Weekday` = 1, `TimeOfDay`, `Holiday`, `Enable` = TRUE, `WorkOn1` = 7 h, `WorkOff1` = 18 h,
`WorkOn2`, `WorkOff2`, `SatOn`, `SatOff`, `SunOn`, `SunOff`, `OnValue` = 1.0, `OffValue` = 0.0.
Ausgänge: `Q`, `Value`.

- Belegungsfenster sind **halboffen** `[On, Off)`: Um 7:00 ist man drin, um 18:00 schon draußen
  (getestet).
- `Off < On`: Das Fenster geht über Mitternacht (`On ≤ t` **oder** `t < Off`) und zählt zu dem Tag, dessen
  Wochentag gerade gilt (getestet: Mi 22:00–02:00, um 01:00 → TRUE).
- `On = Off`: nie belegt. Das ist die Vorgabe für Samstag, Sonntag und das zweite Werktagsfenster
  (getestet).
- Montag–Freitag: Fenster 1 ODER Fenster 2; `Weekday = 6`: Samstag; `Weekday = 7` oder `Holiday`:
  Sonntagsprogramm (getestet). Werte außerhalb 1..7 gelten als Werktag.
- `Enable = FALSE` → `Q = FALSE`, `Value = OffValue`.

**Auslegung:** Ein Fenster über Mitternacht gehört zu beiden Tagen, nicht nur zum Starttag. Freitag
22:00–02:00 belegt also auch Freitag 0:00–2:00, und am Samstag um 1:00 gilt das Samstagsprogramm. Wer das
nicht will, teilt das Fenster selbst auf.

### VDI3814.CAL – Kalender

Eingänge: `Year` = 2026, `Month` = 1, `Day` = 1, `GermanHolidays` = TRUE, `D1`..`D8` (MMTT, 0 = frei),
`RangeFrom`, `RangeTo` (MMTT).
Ausgänge: `Q` (Feiertag oder Sondertag), `Holiday` (gesetzlicher Feiertag), `Special`.

- Bundesweite gesetzliche Feiertage: Neujahr, Karfreitag, Ostermontag, 1. Mai, Christi Himmelfahrt,
  Pfingstmontag, 3. Oktober, 25. und 26. Dezember. Ostern nach dem anonymen gregorianischen Algorithmus
  (getestet für 2026: Karfreitag 3.4., Ostermontag 6.4., Himmelfahrt 14.5.). Länderfeiertage trägt man
  in `D1..D8` ein (Fronleichnam usw. ist beweglich und deshalb jedes Jahr neu einzutragen).
- Ostersonntag und Pfingstsonntag sind keine eigenen Einträge, weil sie ohnehin Sonntage sind
  (getestet).
- Zeitraum `RangeFrom..RangeTo` einschließlich beider Grenzen. Ist `RangeFrom > RangeTo`, geht der Zeitraum
  über den Jahreswechsel (1224..0106, getestet). Er gilt nur, wenn beide Grenzen > 0 sind.
- Ungültige Daten (MMTT = 1340) treffen nie.

### VDI3814.NC – Alarm-/Ereignismeldung

Eingänge: `Event`, `Ack`, `Delay`, `Prio` = 3, `NeedAck` = TRUE.
Ausgänge: `Active`, `Unacked`, `Alarm`, `Lamp`, `State`, `Count`.

Zustandsmodell (die üblichen GA-Meldezustände):

| `State` | Bedeutung | `Active` | `Unacked` | `Lamp` |
|---|---|---|---|---|
| 0 | normal | FALSE | FALSE | aus |
| 1 | kommend, nicht quittiert | TRUE | TRUE | blinkt 1 Hz |
| 2 | kommend, quittiert | TRUE | FALSE | Dauerlicht |
| 3 | gegangen, nicht quittiert | FALSE | TRUE | blinkt |

- `Active` = `Event` hat `Delay` lang angestanden (Ansprechverzögerung, getestet). Das Gehen ist
  unverzögert.
- Jedes Kommen zählt `Count` hoch und setzt `Unacked` (bei `NeedAck`).
- Eine Flanke an `Ack` quittiert. Eine Quittierung **vor** dem Kommen quittiert die künftige Meldung nicht
  vor (getestet). Ein gehaltenes `Ack` gilt nur einmal.
- `Alarm = Active OR Unacked`, für Sammelmeldungen.
- `Prio` (Meldeklasse nach den Meldungsklassen der VDI-Beispielliste: 1 Gefahr für Leben, 2 Sicherheit,
  3 Alarm, 4 Störung, 5 Wartung, 6 System, 7 Handeingriff, 8 Sonstiges) wird nicht ausgewertet. Die
  Bedienoberfläche liest es zum Sortieren und Färben der Meldeliste.
- Das Blinken folgt der SPS-Zeit (500 ms an / 500 ms aus); alle `NC` blinken synchron.

### VDI3814.LOG – Datenaufzeichnung

Eingänge: `In`, `Interval` = 1 min, `Enable` = TRUE, `Clear`.
Ausgänge: `Count`, `Last`, `Min`, `Max`, `Avg`.

- Ringspeicher `Buf[0..59]` (60 Werte, lesbar als `instanz.Buf[i]`); `Next` ist der nächste Platz.
- Der **erste** Aufruf mit `Enable` speichert sofort, danach jeweils nach `Interval` (getestet).
- Statistik über die gespeicherten Werte. Nach 60 Werten wird der älteste überschrieben, `Count` bleibt 60
  (getestet).
- `Clear` (Pegel) leert den Speicher und setzt die Statistik auf 0. Der erste Aufruf danach mit
  `Enable` speichert wieder sofort.
- `Enable = FALSE` hält die Zeitmessung an (keine Lücke wird nachgeholt).
- Für den zeitlichen Verlauf in der Bedienoberfläche ist der Linienschreiber gedacht (Browser-seitig, mit
  Zeitstempeln). `LOG` liefert die Statistik in der SPS selbst.

## 2.1 Logik

### VDI3814.SCALE – lineare Umrechnung (Funktion)

`SCALE(In, InLo, InHi, OutLo, OutHi)` = `OutLo + (In − InLo)·(OutHi − OutLo)/(InHi − InLo)`, ohne
Begrenzung; `InLo = InHi` → `OutLo`.

### VDI3814.MEAN4 – Mittelwert/Minimum/Maximum

Eingänge: `In1`..`In4`, `N` = 2. Ausgänge: `Avg`, `Min`, `Max`.
Ausgewertet werden die ersten `LIMIT(1, N, 4)` Eingänge (getestet: `N = 0` wirkt wie 1).

### VDI3814.TONOF – Ein- und Ausschaltverzögerung

Eingänge: `IN`, `TOn`, `TOff`. Ausgang: `Q`.
`Q` wird TRUE, wenn `IN` `TOn` lang TRUE war, und FALSE, wenn `IN` `TOff` lang FALSE war. Ein kürzerer
Impuls in die jeweilige Gegenrichtung setzt die Verzögerung zurück (getestet). `TOn`/`TOff` = 0: sofort.

## 2.2 Überwachen und Steuern

### VDI3814.LIMITMON – Grenzwertüberwachung

Eingänge: `In`, `High` = 1E30, `Low` = −1E30, `Hyst`, `Delay`, `Enable` = TRUE.
Ausgänge: `QHigh`, `QLow`, `Q`.

- Oben: an bei `In > High` (echt größer), aus bei `In < High − Hyst`. Unten spiegelbildlich: an bei
  `In < Low`, aus bei `In > Low + Hyst`. Die Hysterese liegt also **innerhalb** des Gutbereichs
  (getestet).
- `Delay` verzögert nur das Ansprechen; das Rückfallen ist unverzögert.
- `Enable = FALSE` setzt beide Meldungen sofort zurück (getestet).
- `Hyst < 0` ergibt ein Flattern an der Grenze. Es wird nicht korrigiert, weil es ein Parametrierfehler
  ist.

### VDI3814.RUNHOURS – Betriebsstundenüberwachung

Eingänge: `Run`, `ServiceInterval` [h], `ServiceDone`, `Preset` [h], `SetPreset`.
Ausgänge: `Hours` [h, LREAL], `Starts`, `SinceService` [h], `ServiceDue`.

- `Hours` und `SinceService` zählen, solange `Run` TRUE ist; `Starts` zählt die steigenden Flanken von
  `Run` (getestet).
- Flanke `ServiceDone` → `SinceService := 0`; Flanke `SetPreset` → `Hours := Preset` (Zählertausch).
- `ServiceDue = ServiceInterval > 0 AND SinceService ≥ ServiceInterval`.
- LREAL: Die Auflösung bleibt bei 100 000 h noch weit unter einer Sekunde.
- Die Zählerstände gehen bei Neustart/Warmstart verloren, solange RETAIN nicht unterstützt wird. Für
  echte Betriebsstunden einen Zählerstand per `Preset` zurückschreiben.

### VDI3814.EVENTCOUNT – Ereignisüberwachung (Zählung)

Eingänge: `In`, `Reset`, `Limit`, `Window` = 1 h, `MaxPerWindow`.
Ausgänge: `Count`, `LimitReached`, `InWindow`, `Frequent`.

- `Count` zählt steigende Flanken; `LimitReached = Limit > 0 AND Count ≥ Limit`.
- **Auslegung:** Die Häufigkeit wird in **festen Fenstern** gezählt, nicht gleitend. Nach je `Window`
  beginnt `InWindow` bei 0 (getestet). `Frequent = InWindow > MaxPerWindow` (echt größer, 0 = keine
  Prüfung). Zwei Häufungen an einer Fenstergrenze können so unentdeckt bleiben. Das wurde gegen den
  Speicherbedarf eines gleitenden Fensters abgewogen.
- `Reset` (Pegel) löscht Zähler, Fenster und `Frequent`.

### VDI3814.CMDMON – Befehlsausführüberwachung

Eingänge: `Cmd`, `Feedback`, `OnDelay` = 10 s, `OffDelay` = 10 s, `Reset`.
Ausgänge: `Fault`, `NoFeedback`, `Unexpected`.

- `NoFeedback`: Befehl ohne Rückmeldung länger als `OnDelay`. `Unexpected`: Rückmeldung ohne Befehl
  länger als `OffDelay` (hängendes Schütz, Handbetrieb vor Ort).
- Beide speichern bis zur Flanke `Reset` (getestet). Steht die Ursache bei der Quittierung noch an, setzt
  sie sich nach Ablauf der Verzögerung wieder.

### VDI3814.DEVSTATUS – Anlagen-/Gerätestatus

Eingänge: `Cmd`, `Running`, `Fault`, `Maintenance`, `LocalManual`.
Ausgänge: `Status`, `IsOff`, `IsOn`, `Changing`, `Manual`, `Service`, `Faulted`.

`Status` nach Vorrang: 5 Störung > 4 Wartung > 3 Hand vor Ort > 2 Befehl ≠ Rückmeldung (läuft an/aus) > 1
ein > 0 aus (getestet). Genau einer der Bool-Ausgänge ist TRUE. Ohne Verzögerung: Für eine
Anlaufzeit `Changing` zeitlich bewerten oder `CMDMON` verwenden.

### VDI3814.MOTOR – Motorsteuerung

Eingänge: `Auto`, `Mode` = 2, `Release` = TRUE, `Feedback`, `Overload`, `FbDelay` = 5 s,
`UseFeedback` = TRUE, `Reset`, `MinOff`.
Ausgänge: `Q` (Schütz), `Running`, `Fault`, `FaultCode`, `Status`.

- `Mode`: 0 Aus, 1 Hand Ein, 2 Automatik (dann `Auto`). Andere Werte wirken wie 0.
- Einschaltbedingung: keine Störung, `Release` und Modus fordert ein. Nach dem Ausschalten Wiederanlaufsperre
  `MinOff`. Das erste Einschalten nach dem Start wartet nicht.
- `Release = FALSE` (Verriegelung, Sicherheitskette) schaltet in **jedem** Modus sofort ab (getestet).
- Störungen speichern und schalten ab:
  - `FaultCode` 1 `Overload` (Motorschutz), wirkt sofort und hat Vorrang.
  - 2: Befehl ohne `Feedback` länger als `FbDelay`.
  - 3: `Feedback` ohne Befehl länger als `FbDelay`.
  2 und 3 gibt es nur mit `UseFeedback`. Eine spätere Störung überschreibt den Code einer bestehenden nicht
  (Erstwert, getestet). Nur `Overload` überschreibt jede.
- Flanke `Reset` quittiert, aber **nur ohne anstehendes `Overload`** (getestet). Eine noch
  anstehende Fehlrückmeldung (Code 3) setzt die Störung nach `FbDelay` erneut (getestet).
- Nach der Quittierung läuft ein Motor in Automatik oder Hand **sofort wieder an**, wenn der Modus es
  fordert. **Auslegung:** Das entspricht dem üblichen GA-Verhalten. Soll ein Wiederanlauf nach
  Quittierung nicht automatisch erfolgen, in der Anwendung `Mode` vor der Quittierung auf 0 setzen.
- `Running = Q AND (Feedback OR NOT UseFeedback)`; `Status`: 0 aus, 1 läuft, 2 Störung, 3 Anlauf (Befehl,
  noch ohne Rückmeldung).

### VDI3814.ANTISEIZE – Blockierschutz

Eingänge: `Cmd`, `Enable` = TRUE, `Interval` = 24 h, `Duration` = 30 s, `Inhibit`.
Ausgänge: `Q`, `Exercising`.

- `Q = Cmd OR Exercising`. Die Stillstandszeit zählt, solange `Cmd` FALSE ist; jeder Lauf über `Cmd`
  setzt sie zurück (getestet).
- Nach `Interval` Stillstand (mit `Enable`, ohne `Inhibit`) läuft das Aggregat `Duration` lang. `Inhibit`
  oder `Enable = FALSE` bricht einen Probelauf sofort ab, und die Stillstandszeit beginnt neu.
- Auch ein abgebrochener Probelauf setzt die Stillstandszeit zurück. Der nächste Versuch kommt also nach
  einem vollen `Interval`.

### VDI3814.ASWITCH – Umschaltung (Analogwert)

Eingänge: `Sel`, `In0`..`In3`, `Ramp` [Einheit/s]. Ausgang: `Out`.
`Sel` 1..3 wählt `In1..In3`, **jeder andere Wert** `In0` (getestet). `Out` folgt mit höchstens `Ramp`;
`Ramp ≤ 0` oder erster Aufruf: sofort.

### VDI3814.SEQUENCE – Folgesteuerung

Eingänge: `Demand` [%], `Stages` = 4, `OnDelay` = 2 min, `OffDelay` = 2 min, `Hyst` = 5,
`Rotate`, `Avail1`..`Avail4` = TRUE.
Ausgänge: `Q1`..`Q4`, `Active`, `Lead`.

- `N = LIMIT(1, Stages, 4)`, Stufenbreite `100/N` %. Mit `Active` laufenden Stufen wird eine Stufe
  **zugeschaltet**, wenn `Demand > Active·100/N + Hyst` für `OnDelay` ansteht, und eine **abgeschaltet**,
  wenn `Demand ≤ (Active−1)·100/N` für `OffDelay` ansteht. Pro Verzögerungsablauf ändert sich `Active` um
  genau 1 (getestet: 3 Stufen, Bedarf 50 % → 2 Stufen).
- Die geforderten Stufen werden auf die Aggregate in der Reihenfolge `Lead`, `Lead+1`, … verteilt
  (umlaufend). Nicht verfügbare (`AvailX = FALSE`) werden übersprungen, und das nächste verfügbare springt
  ein (getestet).
- Flanke `Rotate`: das nächste Aggregat führt (Führungswechsel, z. B. täglich aus einem Zeitplan).
- **Auslegung:** `Active` ist der *Bedarf* an Stufen, nicht die Zahl laufender Aggregate. Sind zu wenige
  verfügbar, laufen alle verfügbaren und `Active` bleibt beim Bedarf (getestet). Das zeigt den
  Leistungsmangel an.
- `Stages` während des Betriebs zu verkleinern begrenzt `Active` sofort.

### VDI3814.FROST – Frostschutzsteuerung

Eingänge: `Temp`, `PreLimit` = 8 °C, `TripLimit` = 5 °C, `Hyst` = 2 K, `Thermostat`, `Delay` = 2 s,
`Reset`.
Ausgänge: `Y` [%], `PreAlarm`, `Trip`, `Release`.

- `Y` öffnet das Heizventil zwangsweise: 0 % bei `Temp ≥ PreLimit`, linear bis 100 % bei
  `Temp = TripLimit` (getestet: 6,5 °C → 50 %). Verknüpfung mit dem Regler über `MAX(Regler.Y, Frost.Y)`.
- `Trip`, wenn `Temp < TripLimit` oder der Frostschutzthermostat (`Thermostat = TRUE` = Frost) `Delay`
  lang ansteht. `Trip` speichert: `Y = 100 %`, `Release = FALSE` (Ventilator aus, Außenluftklappe zu).
- Quittierung per Flanke `Reset` nur ohne Thermostat und mit `Temp > TripLimit + Hyst` (getestet: bei
  6 °C abgelehnt, bei 7,5 °C angenommen).
- `PreLimit ≤ TripLimit` (Fehlparametrierung): kein proportionales Öffnen, nur die Auslösung.
- Ein ausgefallener Fühler meldet sich meist mit einem extremen Wert. Eine unplausibel tiefe Temperatur
  löst den Frostschutz aus (sichere Richtung).

### VDI3814.SAFETY – Sicherheitssteuerung

Eingänge: `OK1`..`OK4` = TRUE (TRUE = Kontakt geschlossen = sicher), `Reset`.
Ausgänge: `Release`, `Trip`, `FirstOut`.

- Ein geöffneter Kontakt löst aus und speichert. `FirstOut` ist der Kontakt, der zuerst geöffnet hat;
  mehrere im selben Zyklus: der mit der kleinsten Nummer. Später öffnende ändern `FirstOut` nicht
  (getestet).
- Entriegeln per Flanke `Reset`, nur wenn **alle** Kontakte geschlossen sind (getestet). Ein `Reset`, das
  schon vor dem Schließen anstand, entriegelt nicht. Die Flanke muss danach kommen.
- Keine Sicherheitsfunktion im Sinne der Maschinensicherheit (kein SIL/PL, siehe Spezifikation §1). Die
  Sicherheitskette selbst muss hardwareseitig wirken; der Baustein bildet ihre Meldung und Verriegelung
  in der GA ab.

### VDI3814.PRIORITY – Prioritätssteuerung

Eingänge: `En1`/`V1` … `En8`/`V8`, `Default`. Ausgänge: `Out`, `Q`, `Level`.
Die freigegebene Ebene mit der kleinsten Nummer gewinnt, keine → `Default`, `Level = 0` (getestet).
`Q = Out <> 0`, für binäre Befehle (0/1).
**Auslegung:** wie die Priority-Array-Idee von BACnet, aber mit 8 statt 16 Ebenen und mit expliziter
Freigabe statt „NULL“.

## 2.3 Regelung

### VDI3814.PID – P-/PI-/PID-Regler

Eingänge: `SP`, `PV`, `Kp` = 1, `Tn` [s], `Tv` [s], `Reverse`, `YMin` = 0, `YMax` = 100, `Deadband`,
`Enable` = TRUE, `Manual`, `YManual`, `YOff`, `Offset`.
Ausgänge: `Y`, `E`, `AtLimit`.

Algorithmus (Stellungsalgorithmus, je Zyklus):

1. `E = SP − PV` (bei `Reverse`: `PV − SP`, z. B. Kühlen). `|E| < Deadband` → `E = 0` (Totzone **ohne**
   Sprung am Rand: außerhalb wird E unverändert verwendet).
2. `P = Kp·E`; `I += Kp·E·dt/Tn` (nur mit `Tn > 0`, sonst `I = 0`); `D` = gefilterte Ableitung
   `Kp·Tv·dE/dt`, Filterzeitkonstante `Tv/8` (nur mit `Tv > 0`; der erste Zyklus nach dem Start hat kein D).
3. `Y = LIMIT(YMin, P + I + D + Offset, YMax)`.

Anti-Windup und Umschaltung:
- Der I-Anteil wird jeden Zyklus auf `[YMin − P − Offset, YMax − P − Offset]` begrenzt (Rückrechnung).
  Auch nach langer Sättigung verlässt `Y` die Grenze **sofort**, wenn sich das Vorzeichen von E dreht
  (getestet: < 5 % zwei Zyklen nach Sollwertsprung von 100 auf 0).
- `Enable = FALSE` → `Y = YOff` (begrenzt). `Manual` (mit `Enable`) → `Y = YManual`. In beiden Fällen
  wird `I = Y − P − Offset` nachgeführt, damit die Rückkehr in die Automatik stoßfrei ist (getestet).
- `Offset` ist der Arbeitspunkt des P-Reglers (`Tn = 0`); mit I-Anteil wirkt er nur beim Start.

Grenzfälle:
- `Kp = 0`: P, D und die Integration sind 0, der I-Anteil behält seinen bisherigen Wert (nach dem Start 0,
  also `Y = Offset`).
- Negatives `Kp` kehrt die Wirkrichtung um wie `Reverse`. Mit `Tn > 0` arbeitet die Begrenzung dann
  trotzdem richtig.
- `YMin > YMax`: `Y = YMax` (LIMIT).
- Der D-Anteil wirkt auf die Regelabweichung, also auch auf Sollwertsprünge (mit Filter `Tv/8`
  abgeschwächt). Für sprungfreie Sollwerte vorher `LIMITSP` mit Rampe verwenden.

### VDI3814.SPCURVE – Sollwertführung/-kennlinie

Eingänge: `X`, `X1`/`Y1` = −15/70, `X2`/`Y2` = 0/52, `X3`/`Y3` = 10/38, `X4`/`Y4` = 20/22, `Shift`,
`YMin` = −1E30, `YMax` = 1E30.
Ausgang: `Y`.

Polygonzug durch die vier Punkte, linear dazwischen, **konstant** außerhalb (`X ≤ X1` → `Y1`,
`X ≥ X4` → `Y4`); dann `+ Shift` (Parallelverschiebung) und `LIMIT(YMin, ·, YMax)` (getestet). Die Vorgabe
ist eine Heizkurve Außentemperatur → Vorlauftemperatur.
Grenzfälle: Die X-Werte müssen aufsteigend sein. Zwei gleiche X ergeben eine Stufe (keine Division
durch null, der Abschnitt ist leer). Ein Abschnitt mit fallenden X trifft nie; liegt `X` in keinem
Abschnitt, gilt `Y4`.

### VDI3814.OUTCONT – Stellausgabe stetig

Eingänge: `Y` [%], `YMin` = 0, `YMax` = 100, `OutLo` = 0, `OutHi` = 10, `Invert`, `Safe`, `YSafe`,
`Ramp` [%/s].
Ausgänge: `Out`, `Pos` [%].

- `Pos = LIMIT(max(YMin,0), Y, min(YMax,100))` (Minimal-/Maximalstellung, z. B. Mindestaußenluft); bei
  `Safe` stattdessen `YSafe` (nur auf 0..100 begrenzt). **Die Sicherheitsstellung übergeht YMin/YMax**
  (getestet).
- `Ramp > 0`: Änderungsgeschwindigkeit; erster Aufruf ohne Rampe.
- `Out = OutLo + Pos/100·(OutHi − OutLo)`; `Invert`: `Out = OutHi − Pos/100·(OutHi − OutLo)`. Für
  2..10 V: `OutLo = 2`.

### VDI3814.OUT2P – Stellausgabe 2-Punkt

Eingänge: `Y`, `OnAt` = 60, `OffAt` = 40, `MinOn`, `MinOff`. Ausgang: `Q`.
Ein bei `Y ≥ OnAt`, aus bei `Y ≤ OffAt`, jeweils erst nach der Mindestzeit im bisherigen Zustand (getestet).
Das erste Einschalten wartet nicht auf `MinOff`.
**Auslegung:** Wie alle „Stellausgaben“ der VDI 3814 setzt OUT2P eine **Stellgröße** um (z. B. die eines
PID für eine einstufige Heizung) und ist selbst kein Zweipunktregler. Einen Thermostaten bildet man mit
`LIMITMON` (Grenzwert + Hysterese) oder `OUT2P` hinter einem P-Regler.
`OnAt ≤ OffAt` ergibt Flattern zwischen beiden Werten und ist ein Parametrierfehler.

### VDI3814.OUT3P – Stellausgabe 3-Punkt

Eingänge: `Y` [%], `RunTime` = 120 s, `Deadband` = 2 %, `MinPulse` = 200 ms, `SyncExtra` = 10 %.
Ausgänge: `Open`, `Close`, `Pos` [%], `Syncing`.

Ein Antrieb ohne Stellungsrückmeldung: Der Baustein bildet die Stellung aus den Fahrzeiten nach
(100 % in `RunTime`).

- **Start-Synchronisation:** Der erste Aufruf fährt `Close` für `RunTime·(1 + 2·SyncExtra/100)`. Die
  Stellung ist unbekannt, deshalb wird von „ganz offen + Überhub“ bis „zu + Überhub“ gefahren. Danach ist
  `Pos = 0` und `Syncing = FALSE` (getestet). `Y` wird währenddessen nicht beachtet.
- **Positionieren:** Eine Fahrt beginnt, wenn `|Y − Pos| > Deadband`, und endet, wenn die Modellstellung
  das Ziel erreicht oder überschritten hat, frühestens nach `MinPulse` (getestet: Y 50 → 51 bei 2 % Totzone:
  keine Fahrt).
- **Endlagen-Synchronisation:** `Y ≥ 100` (bzw. `≤ 0`) fährt um `SyncExtra` % der Laufzeit über die
  Endlage hinaus, damit Modell und Antrieb sicher übereinstimmen. Danach ist `Pos = 100` (bzw. 0), und
  **solange Y in der Endlage bleibt, wird nicht erneut gefahren** (Regressionstest: vorher endloses „Auf“,
  siehe Test „OUT3P“).
- `Open` und `Close` sind nie gleichzeitig TRUE. Ein Richtungswechsel geht immer über einen Zyklus Stillstand.
- Eine Fahrt wird nicht unterbrochen, wenn sich `Y` währenddessen ändert. Sie endet am alten Ziel (oder
  dahinter), danach folgt die Korrektur. Das schont das Getriebe.

Grenzfälle: `RunTime` unter 0,1 s zählt als 0,1 s. `Deadband = 0` mit `MinPulse > 0` erzeugt Pendeln um
den Sollwert in Schritten von `MinPulse`, deshalb immer eine Totzone von mindestens
`MinPulse/RunTime·100` % wählen.

### VDI3814.OUT3PVAR – Stellausgabe 3-Punkt mit variabler Impulslänge

Eingänge: `E` (Regelabweichung, positiv = auf), `Kp` = 1 [% Hub je Einheit E und Periode],
`RunTime` = 120 s, `Period` = 10 s, `MinPulse` = 200 ms, `Deadband` = 0,5.
Ausgänge: `Open`, `Close`, `Pulse`.

Zu Beginn jeder `Period` (der erste Aufruf beginnt eine) wird einmal entschieden:
`|E| ≤ Deadband` → kein Impuls; sonst Richtung nach dem Vorzeichen von E, Länge
`LIMIT(MinPulse, |E|·Kp/100·RunTime, Period)` (getestet: E = 5, Kp = 1, 120 s → 6 s; sehr große E → ganze
Periode; sehr kleine E → MinPulse). Der Impuls liegt am Anfang der Periode. Eine Änderung von E während
der Periode wirkt erst ab der nächsten. Ergibt ein Schrittregler mit PI-ähnlichem Verhalten (Antrieb
integriert).

### VDI3814.PWM – Stellausgabe Pulsweitenmodulation

Eingänge: `Y` [%], `Period` = 10 s, `MinPulse`, `Enable` = TRUE. Ausgänge: `Q`, `OnTime`.

- Am Anfang jeder Periode: `OnTime = LIMIT(0,Y,100)/100·Period`. Ist `OnTime < MinPulse` → 0. Ist
  `Period − OnTime < MinPulse` → ganze Periode (getestet für 5 % und 95 %).
- `Q` ist am Periodenanfang `OnTime` lang TRUE (getestet: 30 % von 1 s → 6 von 20 Zyklen à 50 ms).
- Die Auflösung ist die Zykluszeit: Bei 50 ms Zyklus und 1 s Periode gibt es 20 Stufen.
- `Enable = FALSE` schaltet `Q` ab; die Periode läuft weiter.

### VDI3814.LIMITSP – Begrenzung Sollwert/Stellwert

Eingänge: `In`, `Lo` = −1E30, `Hi` = 1E30, `RateUp`, `RateDown` [Einheit/s].
Ausgänge: `Out`, `AtLo`, `AtHi`, `Limited`.
`Out` folgt `LIMIT(Lo, In, Hi)` mit höchstens `RateUp`/`RateDown` (0 = unbegrenzt). Der erste Aufruf
springt. `AtLo`/`AtHi`: der **Eingang** liegt außerhalb. `Limited`: `Out ≠ In` (Grenze oder Rampe aktiv).

### VDI3814.PARAMSET – Parameterumschaltung

Eingänge: `Sel` = 1, `A1`/`B1`/`C1` … `A4`/`B4`/`C4`. Ausgänge: `A`, `B`, `C`.
`Sel` 2..4 wählt den Satz, **jeder andere Wert Satz 1** (getestet). Ohne Überblendung: Für stoßfreies
Umschalten der Reglerparameter sorgt der PID selbst (I-Anteil-Begrenzung), ein Sprung von `Kp` springt
aber den P-Anteil.

## 2.4 Optimierung

### VDI3814.ENERGYLEVEL – Energieniveau

Eingänge: `Level` = 1, `Presence`, `Window`, `HeatComfort` = 21, `CoolComfort` = 24, `PreShift` = 2,
`EcoShift` = 4, `ProtHeat` = 7, `ProtCool` = 35, `Shift`.
Ausgänge: `HeatSP`, `CoolSP`, `Active`.

| `Active` | Niveau | `HeatSP` | `CoolSP` |
|---|---|---|---|
| 1 | Komfort | HeatComfort + Shift | CoolComfort + Shift |
| 2 | Pre-Komfort | HeatComfort + Shift − PreShift | CoolComfort + Shift + PreShift |
| 3 | Economy | HeatComfort − EcoShift | CoolComfort + EcoShift |
| 4 | Gebäudeschutz | ProtHeat | ProtCool |

- `Active = LIMIT(1, Level, 4)` (getestet: 9 → 4); offenes `Window` → 4; `Presence` hebt **nur**
  Pre-Komfort auf Komfort, nicht Economy (Nacht, getestet).
- **Auslegung** nach VDI 3813 (Raumautomation): Die Sollwertverschiebung des Nutzers (`Shift`) wirkt nur
  bei Komfort und Pre-Komfort. Liegt `HeatSP` durch eine große Verschiebung über `CoolSP`, wird das nicht
  korrigiert; die nachfolgende Regelung muss eine Totzone erzwingen.

### VDI3814.HEATRECOVERY – Energierückgewinnung

Eingänge: `Toa` (Außenluft), `Tex` (Abluft), `SupplySP`, `Efficiency` = 0,7, `IcingLimit` = −5,
`IcingY` = 50, `Enable` = TRUE.
Ausgänge: `Y` [%], `Heating`, `Cooling`, `Tsupply`.

- Rückgewinnung nutzt nur, wenn Abluft und Sollwert auf **derselben Seite** der Außenluft liegen:
  Heizen (`SupplySP > Toa` und `Tex > Toa`) oder Kühlen (beide kleiner). Sonst `Y = 0` (getestet).
- Benötigt wird `Y = (SupplySP − Toa)/(η·(Tex − Toa))·100`, begrenzt 0..100 (Kühlen spiegelbildlich).
  `η` ist die Temperaturübertragung bei 100 %, mit linearer Leistung über Y angenommen (getestet: Toa 0,
  Tex 22, η 0,7, Soll 7,7 → 50 %).
- Unter `IcingLimit` höchstens `IcingY` (Vereisungsschutz, getestet).
- `Tsupply` = erwartete Lufttemperatur nach der WRG (für die Nachheizung).
- `Efficiency` wird auf 0,05..1 begrenzt.

### VDI3814.HXSETPOINT – h,x-Sollwertführung

Eingänge: `T` [°C], `RH` [%], `Pressure` = 1013,25 hPa, `XMin` = 6 g/kg, `XMax` = 11,5 g/kg,
`Hyst` = 0,3 g/kg.
Ausgänge: `X` [g/kg], `H` [kJ/kg], `DewPoint` [°C], `XSet`, `RHSet` [%], `Humidify`, `Dehumidify`.

- Sättigungsdampfdruck nach Magnus über Wasser (6,112 hPa·e^(17,62·T/(243,12+T))), auch unter 0 °C (über
  Eis weicht das um bis zu 10 % ab).
- `X = 622·pw/(p − pw)`, `H = 1,006·T + X/1000·(2501 + 1,86·T)`, Taupunkt aus der Umkehrung der
  Magnusformel (getestet: 20 °C/50 % → 7,26 g/kg, 38,5 kJ/kg, 9,3 °C).
- **Behaglichkeitsfeld:** `XSet = LIMIT(XMin, X, XMax)`. Innerhalb des Feldes wird weder be- noch
  entfeuchtet. `Humidify` ab `X < XMin − Hyst`, aus bei `X ≥ XMin`; `Dehumidify` spiegelbildlich.
  `RHSet` ist die relative Feuchte, die `XSet` bei der aktuellen Temperatur hätte (für Regler mit
  rF-Fühler, getestet: 20 °C, 20 % → 41,5 %).
- `RH` wird auf 0,001..100 % begrenzt (kein Logarithmus von 0).

### VDI3814.OPTSTART – Schaltzeitpunktoptimierung

Eingänge: `TimeOfDay`, `OccStart` = 7 h, `Troom`, `SP` = 21, `Rate` = 2 K/h, `MaxLead` = 3 h,
`Learn` = TRUE.
Ausgänge: `Start`, `Lead`, `LearnedRate`.

- `Lead = LIMIT(0, (SP − Troom)/LearnedRate, MaxLead)`; `Start` wird TRUE, wenn die Restzeit bis
  `OccStart` in `(0, Lead]` liegt und der Raum kälter als `SP` ist (getestet: 17 °C, 2 K/h → 2 h vor
  Beginn). Es bleibt TRUE bis `OccStart`, auch wenn `SP` vorher erreicht wird. Ab dann übernimmt der
  Zeitplan.
- Die Restzeit rechnet über Mitternacht (Nutzung 0:30, jetzt 23:00 → 1,5 h).
- Lernen: Bei jedem Aufheizen wird die Zeit vom Start bis `Troom ≥ SP` gemessen und
  `LearnedRate := 0,7·alt + 0,3·gemessen` (begrenzt 0,1..50 K/h, getestet). Aufheizvorgänge unter 3 min
  oder ohne Temperaturanstieg zählen nicht.
- **Auslegung:** nur Aufheizen (Winter). Für Vorkühlen eine zweite Instanz mit vertauschtem Vorzeichen
  (`SP := −Kühlsollwert`, `Troom := −Raumtemperatur`) verwenden.

### VDI3814.NIGHTCOOL – Nachtkühlung

Eingänge: `Night`, `Troom`, `Toa`, `TroomStart` = 23, `TroomMin` = 18, `DeltaT` = 3, `ToaMin` = 10,
`Hyst` = 1.
Ausgang: `Q`.
Ein bei `Night`, `Troom > TroomStart`, `Troom − Toa > DeltaT + Hyst` und `Toa > ToaMin`. Aus, sobald eins
gilt: Nacht vorbei, `Troom < TroomMin`, `Troom − Toa < DeltaT`, `Toa < ToaMin − Hyst` (getestet). Regen
oder Wind (Fenster) verknüpft man außen mit `WEATHERPROT`.

## 2.5 Beleuchtung

### VDI3814.LIGHT – Lichtsteuerung

Eingänge: `Button`, `On`, `Off`, `Presence` = TRUE, `AbsenceDelay` = 15 min, `LongPress` = 500 ms,
`DimSpeed` = 25 %/s, `MinLevel` = 10 %, `OnLevel` = 100 %.
Ausgänge: `Q`, `Level` [%].

- Kurzer Druck (losgelassen vor `LongPress`): umschalten; Ein mit `OnLevel` (getestet).
- Langer Druck: dimmen, solange gedrückt, im Bereich `MinLevel..100`. Ist das Licht aus, schaltet es mit
  `MinLevel` ein und dimmt hoch. Bei 100 % dimmt es herunter, bei `MinLevel` hoch, dazwischen entgegen dem
  letzten langen Druck (getestet).
- Zentral: Flanke `On` → ein mit `OnLevel`, Flanke `Off` → aus. Bei gleichzeitigem Auftreten gewinnt
  `Off`.
- `Presence = FALSE` für `AbsenceDelay` → aus. Ohne Präsenzmelder `Presence` unverbunden lassen (TRUE).
- `Level = 0`, wenn aus. **Auslegung:** Die Helligkeit wird beim Ausschalten nicht gespeichert; Ein
  bringt immer `OnLevel`.

### VDI3814.STAIRLIGHT – Treppenlichtschaltung

Eingänge: `Trigger`, `Duration` = 3 min, `Warn` = 30 s, `LongPress` = 2 s.
Ausgänge: `Q`, `Warning`, `Permanent`, `Remaining`, `Level`.

- Flanke `Trigger` → Licht für `Duration`; jede weitere Flanke startet die Zeit neu (Nachtriggern,
  getestet).
- Ausschaltvorwarnung (vgl. DIN 18015-2): In den letzten `Warn` vor dem Ende ist `Warning` TRUE, das Licht
  geht zu Beginn der Warnzeit **500 ms aus** (Blinkzeichen) und leuchtet danach mit `Level = 50 %`
  (für Dimmer, getestet).
- Druck länger als `LongPress` → Dauerlicht (`Permanent`), keine Zeit, keine Warnung. Die **nächste**
  Flanke beendet Dauerlicht und schaltet aus (getestet).
- Das Blinkzeichen liegt dort, wo die Restzeit zwischen `Warn` und `Warn − 500 ms` liegt. Bei
  `Warn ≥ Duration` ist die Warnung von Anfang an aktiv, und das Blinkzeichen kommt nur, soweit diese
  Spanne in die Laufzeit fällt (bei `Warn ≥ Duration + 500 ms` gar nicht).

### VDI3814.DAYLIGHT – Tageslichtschaltung

Eingänge: `Lux`, `OnBelow` = 300, `OffAbove` = 500, `Delay` = 1 min. Ausgang: `Q`.
Ein, wenn `Lux < OnBelow` ununterbrochen `Delay` lang; aus, wenn `Lux > OffAbove` `Delay` lang
(getestet). Dazwischen bleibt der Zustand. Startzustand aus.

### VDI3814.CONSTLIGHT – Konstantlichtregelung

Eingänge: `Enable` = TRUE, `Lux`, `SP` = 500, `Ki` = 0,02 %/(lx·s), `MinLevel` = 10, `OffDelay` = 5 min.
Ausgänge: `Q`, `Level` [%].

- I-Regler: `Level += Ki·(SP − Lux)·dt`, begrenzt `MinLevel..100` (getestet: regelt die Raumhelligkeit
  auf den Sollwert).
- Aus, wenn `Level` am Minimum steht und `Lux > 1,1·SP` `OffDelay` lang (genug Tageslicht). Ein, wenn
  `Lux < 0,9·SP`, mit `MinLevel` (getestet).
- `Enable = FALSE` → aus. **Auslegung:** Das Ein/Aus durch Taster/Präsenz erledigt `LIGHT`; `CONSTLIGHT`
  regelt, solange es freigegeben ist (z. B. `Enable := light.Q`).

## 2.6 Sonnenschutz

### VDI3814.SUNBLIND – Sonnenschutz stellen

Eingänge: `AutoPos`, `AutoSlat`, `ManPos`, `ManSlat`, `ManSet`, `ManTimeout` = 2 h, `Protect`, `ProtPos`,
`TravelTime` = 60 s, `SlatTime` = 1,5 s.
Ausgänge: `Pos`, `Slat`, `Up`, `Down`, `ActPos`, `ActSlat`, `Mode`.

Positionen in %: Höhe 0 = oben/offen, 100 = unten/geschlossen; Lamelle 0 = offen (waagerecht),
100 = geschlossen.

- **Priorität:** `Protect` (Witterungsschutz → `ProtPos`, Lamelle offen, `Mode` 2) > Hand (`Mode` 1) >
  Automatik (`Mode` 0), getestet.
- Hand: Flanke `ManSet` übernimmt `ManPos`/`ManSlat`. Zurück in Automatik nach `ManTimeout`. Bei
  `ManTimeout = 0`, sobald sich `AutoPos` oder `AutoSlat` ändert (die Automatik „übernimmt beim nächsten
  Ereignis“). Ein Schutzbefehl beendet den Handbetrieb nicht; danach gilt die Hand weiter, falls die Zeit
  nicht abgelaufen ist.
- **Antriebsmodell (Raffstore):** Beim Abwärtsfahren schließen zuerst die Lamellen (`SlatTime` für
  0→100 %), dann bewegt sich der Behang (`TravelTime` für 0→100 %). Beim Aufwärtsfahren öffnen zuerst die
  Lamellen (getestet). Nach Erreichen der Höhe werden die Lamellen mit kurzen Auf/Ab-Impulsen auf `Slat`
  gestellt (Toleranz 1 %). Ganz oben (`ActPos ≈ 0`) werden die Lamellen nicht gestellt.
- `Up`/`Down` sind die Relaisausgänge (nie beide); `ActPos`/`ActSlat` die nachgebildete Stellung.
  Der Baustein hat keine Rückmeldung und synchronisiert sich bei jeder Fahrt nach 0 bzw. 100 durch
  Begrenzung. Für Antriebe mit Positionsrückmeldung `ActPos` ignorieren.

### VDI3814.SUNDUSK – Sonnen-/Dämmerungsautomatik

Eingänge: `Lux`, `SunOn` = 30 000, `SunOff` = 20 000, `SunOnDelay` = 2 min, `SunOffDelay` = 15 min,
`DuskBelow` = 50, `DuskAbove` = 100, `DuskDelay` = 5 min.
Ausgänge: `Sun`, `Dusk`.
Zwei unabhängige Schwellwertschalter mit Schaltdifferenz; jeder Zustandswechsel braucht die jeweilige
Bedingung ununterbrochen über die Verzögerung (getestet). Wolken unterhalb `SunOffDelay` lassen den
Sonnenschutz also unten.

### VDI3814.THERMOAUTO – Thermoautomatik

Eingänge: `Occupied`, `Sun`, `Troom`, `HeatLimit` = 20, `CoolLimit` = 25, `Hyst` = 0,5.
Ausgänge: `Active`, `Close`, `Open`, `Pos`.
Nur bei Abwesenheit **und** Sonne: `Close` ab `Troom > CoolLimit` (aus unter `CoolLimit − Hyst`), `Open`
ab `Troom < HeatLimit` (aus über `HeatLimit + Hyst`). `Pos` = 100 bei `Close`, sonst 0. Anwesenheit oder
keine Sonne beendet sofort (getestet). Ohne Sonne passiert nichts, weil Schließen dann nicht kühlt und Öffnen
nicht heizt. `Active` dient als Freigabe einer Prioritätsebene, z. B. `PRIORITY.En6 := thermo.Active`.

### VDI3814.WEATHERPROT – Witterungsschutz

Eingänge: `Wind` [m/s], `WindLimit` = 10, `WindHyst` = 3, `Rain`, `RainProtect`, `Toa` = 10,
`FrostLimit` = 2, `Release` = 10 min.
Ausgänge: `Protect`, `WindAlarm`, `RainAlarm`, `FrostAlarm`.

- Wind: an über `WindLimit`, aus unter `WindLimit − WindHyst` (getestet). Regen: `RainAlarm = Rain`.
- Frost (Festfrieren): „nass“ wird mit Regen gesetzt und erst gelöscht, wenn `Toa > FrostLimit + 2 K` ohne
  Regen. `FrostAlarm = nass AND Toa < FrostLimit` (getestet). Frost ohne vorherigen Niederschlag löst also
  nicht aus.
- `Protect` = Wind ODER Frost ODER (Regen und `RainProtect`, z. B. Markisen); das Ende wird um `Release`
  verzögert, das Ansprechen nicht (getestet).
- Ein ausgefallener Windsensor (0 m/s) wird nicht erkannt. Die Überwachung gehört an den `AI` (Fault)
  und als `Protect`-Quelle ODER-verknüpft.

### VDI3814.SUNPOS – Sonnenstandsberechnung

Eingänge: `DayOfYear` = 172, `TimeOfDay` = 12 h, `UtcOffset` = 1 h, `Latitude` = 51°, `Longitude` = 10°.
Ausgänge: `Elevation` [°], `Azimuth` [°, 0 = Nord, 90 = Ost, 180 = Süd], `Day`.

- NOAA-Näherung (Bruchjahr, Zeitgleichung, Deklination), Genauigkeit etwa 0,5° (getestet gegen den
  NOAA-Rechner: Berlin, 21. Juni, wahrer Mittag 13:08 MESZ → Höhe 60,9°, Azimut 180°).
- `UtcOffset` muss zur eingestellten Uhr passen: 1 im Winter (MEZ), 2 im Sommer (MESZ).
- Keine Refraktionskorrektur: Nahe dem Horizont ist die wahre Sonne bis 0,6° tiefer, als die
  scheinbare aussieht. `Day = Elevation > 0`.
- Polarnacht/Mitternachtssonne sind abgedeckt (der Cosinus wird auf −1..1 begrenzt).
