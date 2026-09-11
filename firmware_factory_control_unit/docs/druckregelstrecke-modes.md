# Druckregelstrecke -- Betriebsmodi (Spezifikation)

Status: **Implementiert** (`web/src/apps/druckregelstrecke-app.ts`,
`druckregelstrecke-controller.ts`, `druckregelstrecke-charts.ts`,
`druckregelstrecke-view.ts`; Firmware-seitig `best_binary_buffers_schema/pneumatics.cs` +
`Core/Src/webserver.cpp`/`io.cpp`). Diese Datei bleibt trotzdem die Referenz-Spezifikation --
bei Aenderungswuenschen zuerst hier anpassen, dann den Code nachziehen.

Ergaenzend zur urspruenglichen Beschreibung waehrend der Umsetzung hinzugekommen (unten jeweils an
der passenden Stelle vermerkt):
- Ein **Sollwert** (Ziel-Druck, Rohwert) im Reglerbetrieb-Modus -- ohne ihn haette der Regler
  keine Regelabweichung berechnen koennen; in der urspruenglichen Modus-Liste schlicht vergessen.
- Eine neue, kompakte WebSocket-Nachricht `pneumatics.PressureControlFeedback`
  (Druck-Rohwert + Kompressor-Promille + 3 Ventilzustaende, alle 500ms unbedingt von der Firmware
  gesendet, s. `best_binary_buffers_schema/pneumatics.cs`) speist Trend-Ringpuffer,
  Kennlinien-Laufzeit-Maximum, Sprungantwort-Aufzeichnung UND den Reglerkreis -- unabhaengig vom
  bestehenden 1Hz-`/api/registers`-Polling, das weiterhin nur die Anlagengrafik treibt.

## Umschaltung

Oberhalb der Anlagenvisualisierung sitzt eine Modus-Umschaltung mit vier Optionen:

1. **Freies Experiment** (Standard/Startmodus)
2. **Kennlinie**
3. **Sprungantwort**
4. **Reglerbetrieb**

Unterhalb der Visualisierung wird jeweils die zum aktiven Modus passende Bedien-/Auswerte-UI
eingeblendet.

## 1. Freies Experiment (Standard)

- Liniendiagramm mit zwei Kurven ueber die letzten 60 Sekunden:
  - Kompressorleistung (Promille bzw. daraus abgeleitete Groesse)
  - Druck im Kessel (aktuell Rohwert, s. PRESSURE_RAW-Kalibrierungshinweis in
    `druckregelstrecke-view.ts`)
- Keine weiteren Bedienelemente noetig -- Kompressor/Ventile bleiben ueber die Anlagengrafik
  bedienbar wie bisher.

## 2. Kennlinie

- Kennfelddiagramm:
  - Hochachse: maximal erreichter Druck
  - Rechtsachse (X-Achse): Kompressorleistung
- Fuer jede Ventilkombination (welche der 3 Ventile offen/zu sind) wird eine eigene Kennlinie
  gezeichnet.
- Eine Kennlinie stellt fuer die jeweilige Ventilkombination den bei jeder Kompressorleistung
  maximal erreichten Druck dar.
- **Kein automatischer Sweep** -- das Erzeugen der Kennlinie ist Aufgabe des Nutzers: er stellt
  manuell eine Ventilkombination und eine Kompressorleistung ein, laesst den Kompressor laufen,
  bis sich der Druck eingeschwungen/den Maximalwert erreicht hat, und druckt dann die
  Schaltflaeche **"Wert in Kennliniendiagramm uebernehmen"**. Das setzt bzw. aktualisiert genau
  den Punkt (Kompressorleistung, aktuell beobachteter Maximaldruck) auf der Kennlinie der gerade
  aktiven Ventilkombination -- ein erneutes Uebernehmen fuer dieselbe Kombination+Leistung
  ueberschreibt den vorhandenen Punkt.
- Datenhaltung rein clientseitig (kein Server-/Firmware-Speicher) -- die gesammelten Kennlinien-
  Punkte leben nur im Browser-Zustand der App und gehen beim Neuladen der Seite verloren, sofern
  nicht spaeter bewusst eine Persistenz ergaenzt wird.

## 3. Sprungantwort

Bedienelemente:

- Definition des Sprungs: Vorher-Wert und Nachher-Wert (Kompressorleistung).
- Schaltflaeche **"Start"**: legt den Vorher-Wert am Kompressor an (Einschwingen auf
  Ausgangszustand).
- Schaltflaeche **"Sprung!"**: legt den Nachher-Wert am Kompressor an und startet die
  Werteaufzeichnung.
- Schaltflaeche **"Ende und Analyse"**:
  - schaltet den Kompressor aus,
  - beendet die Aufzeichnung,
  - ermittelt Totzeit und Zeitkonstante der Sprungantwort:
    - **Totzeit**: Zeitpunkt der ersten merklichen Druckveraenderung nach dem Sprung.
    - **Zeitkonstante**: Zeitpunkt, an dem die Sprungantwort 63 % der Gesamtbewegung (Differenz
      zwischen Vorher- und Nachher-Beharrungswert) erreicht hat.

Anzeige:

- Liniendiagramm mit Kompressorleistung und Druck im Kessel der letzten 60 Sekunden (wie im
  Modus "Freies Experiment").

Regler-Entwurfs-Assistent:

- Unterstuetzt zwei Entwurfsverfahren: **T-Summen-Regel** (nach Kuhn, normale/schnelle
  Einstellung) und **Chien-Hrones-Reswick** (0%/20% Ueberschwingen, Fuehrungs-/Stoerverhalten),
  je fuer P/PI/PID -- exakte Formeln in `druckregelstrecke-controller.ts`
  (`designTSumme()`/`designChr()`), Quelle: Kahlert/Bate, "PID-Einstellregeln" (FH Dortmund,
  WS 2008/09), S. 9.
- Streckenkennwerte aus der Sprungantwort: K_S = ΔDruck/ΔLeistung, Totzeit T_u, Zeitkonstante T
  (63%-Zeit NACH Ablauf der Totzeit). **Wichtige Vereinfachung:** Chien-Hrones-Reswick erwartet
  eigentlich T_u/T_g aus dem Wendetangentenverfahren; hier wird T_g durch die gemessene
  Zeitkonstante T angenaehert -- fuer ein reines PT1+Totzeit-Streckenmodell (angenommen fuer
  diesen Versuchsaufbau) sind beide Groessen identisch (Herleitung: bei einer reinen
  Exponentialantwort liegt die Wendetangente direkt am Ende der Totzeit an und erreicht den
  Endwert exakt nach T).
- Ergebnis (K_P, T_N, T_V) per Klick in den Modus "Reglerbetrieb" uebernehmbar (setzt dort auch
  den gewaehlten Reglertyp).

## 4. Reglerbetrieb

Der Regler laeuft **im Browser** (clientseitige JS-Regelschleife), nicht in der Firmware: die
Firmware bleibt ein reines Modbus-Register-Interface (Druck lesen, Kompressorleistung schreiben),
die eigentliche Regelalgorithmik (P/PI/PID, Anti-Windup) sitzt in der Web-App.

- **Zykluszeit: 500 ms** -- deutlich schneller als die erwartete Streckendynamik (s. u.), damit
  die zeitdiskrete Umsetzung die Strecke gut genug abtastet.
- Erwartete Streckenparameter (grobe Erwartung/Richtwert, keine Messung): Totzeit T_t ≈ 3 s,
  Zeitkonstante T ≈ 10 s. Dienen als Sanity-Check fuer die in der Sprungantwort-Analyse
  ermittelten Werte und als Ausgangspunkt, falls noch keine eigene Sprungantwort aufgenommen
  wurde.

Bedienelemente:

- **Sollwert** (Ziel-Druck, Rohwert) -- s. Hinweis oben, in der urspruenglichen Beschreibung
  gefehlt, aber notwendig.
- Umschalter **"Regler ein"** / **"Regler aus"**.
- **Typ des Reglers**: P, PI, PID.
- **Arbeitspunkt des Kompressors**: Grundleistung, die zum Reglerausgang addiert wird.
- **K_P**, **T_N**, **T_V**: nur die fuer den gewaehlten Reglertyp jeweils relevanten Werte sind
  aktiv/editierbar (z. B. bei P-Regler nur K_P, T_N/T_V ausgeblendet).
- **Anti-Windup-Verfahren**: einzige Option **"Begrenzung des Integrators auf 100%"** -- der
  I-Anteil wird in Stellgroessen-Einheiten (Promille) gefuehrt und nach jedem Regelzyklus auf den
  vollen Stellbereich [0, 1000] geklemmt (klassisches Integrator-Clamping, s.
  `PidController` in `druckregelstrecke-controller.ts`).

Anzeige:

- Liniendiagramm (Kompressorleistung + Druck im Kessel, letzte 60 Sekunden) daneben oder darunter,
  analog zu den anderen Modi.

## Umgesetzte Entscheidungen (vormals "Offene Punkte")

- **Anti-Windup:** Begrenzung des Integrators auf 100%, einzige Option (s. Abschnitt 4).
- **Schneller Messwertzugriff fuer den 500ms-Regelkreis:** geloest durch die neue
  `pneumatics.PressureControlFeedback`-WS-Nachricht (s. Kopf dieser Datei) statt eines schnelleren
  `/api/registers`-Pollings.
- **Datenhaltung der 60-Sekunden-Ringpuffer:** rein clientseitig, ein gemeinsamer Ringpuffer
  (`trendSamples` in `druckregelstrecke-app.ts`), gespeist aus der WS-Nachricht.
- **State-Uebergabe zwischen Modi:** App-Ebene (`druckregelstrecke-app.ts`), wie entschieden --
  gilt fuer Kennlinien-Punkte, Sprungantwort-Ergebnis/Entwurfswahl und Reglereinstellungen.

## Bekannte Vereinfachungen / moegliche Folgearbeit

- Die Chien-Hrones-Reswick-Formeln nutzen die gemessene Zeitkonstante T als Naeherung fuer die
  Wendetangenten-Ausgleichszeit T_g (s. Abschnitt 3) -- exakt nur fuer eine reine
  PT1+Totzeit-Strecke; bei staerker "S-foermiger" (Mehrfachverzoegerungs-)Realantwort waere das
  klassische Wendetangentenverfahren (Tangente am Wendepunkt) genauer, ist hier aber nicht
  implementiert.
- Totzeit-Erkennung ueber eine feste Schwelle (2% der Gesamtbewegung, min. 20 Counts) -- bei sehr
  verrauschtem Drucksignal ggf. nachjustieren.
- Kennlinien-Punkte/Sprungantwort-Aufzeichnung/Reglereinstellungen sind reiner In-Memory-State und
  gehen beim Neuladen der Seite verloren (bewusst so entschieden, s. Abschnitt 2).
