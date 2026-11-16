# Bedienen & Beobachten (HMI)

Die Bedien- und Beobachtungsoberfläche gehört zum SPS-Projekt (VDI 3814 Gruppe 3: Bedien-, Anzeige- und
Managementfunktionen). Sie besteht aus

- dem **Editor** in der SPS-Seite der Web-UI (Projektbaum → *Visualisierung*): Seiten gestalten, Elemente
  an SPS-Variablen binden;
- der **Laufzeit** als eigene Seite der Web-UI (*Bedienen & Beobachten*, `/hmi`): die Seiten bedienen, mit
  Meldeliste und Linienschreibern, auch im Vollbild auf einem Bedien-PC.

Code: `web/src/plc/hmi/` (`hmi.ts` Modell, `hmi-view.ts` Darstellung, `hmi-editor.ts`,
`hmi-runtime-app.ts`, `hmi-trends.ts`), Tests `hmi.test.ts`, `hmi-view.test.ts`.

## Speicherung

Die Seiten stehen im Projekt (`project.hmi`, Format in `hmi.ts`). Sie werden mit dem Projekt im Browser
gespeichert, exportiert/importiert und bei „Laden“ auf die SD-Karte der FCU geschrieben. Sie werden nicht
übersetzt und ändern den Projekt-Hash nicht: Eine geänderte Seite macht das Programm auf dem Gerät nicht
„veraltet“.

## Variablen

Ein Element wird über einen **Variablenpfad** gebunden, wie in der Watchliste: globale Variable (`Enable`),
I/O-Variable (`ConveyorPwm`), Programmvariable (`PLC_PRG.Sollwert`), Bausteinausgang
(`PLC_PRG.Regler.Y`, `PLC_PRG.Kessel.Pressure`), Feldelement (`PLC_PRG.Werte[3]`). Groß-/Kleinschreibung
spielt keine Rolle. Die Auswahlliste im Editor enthält alle einfachen Variablen der letzten Übersetzung
(Bausteinmitglieder bis drei Ebenen tief, Felder bis 16 Elemente; interne Variablen der Bibliotheken mit
`__` nicht). Der Editor markiert Pfade, die es im Programm nicht gibt; zur Laufzeit hat ein solches
Element einen rot gestrichelten Rahmen und schreibt nichts.

Die Adressen kommen aus den Debug-Informationen des Programms. Die Laufzeit übersetzt deshalb das Projekt
selbst. Läuft auf dem Gerät ein Programm aus einem anderen Projektstand, zeigt sie „Programm ≠ Projekt“,
und es gibt keine Werte (die Adressen könnten falsch sein).

**Schreiben** geht in das Datensegment der SPS zwischen zwei Zyklen, genau wie „Wert schreiben“ in der
Watchliste:

- Schreibt das Programm dieselbe Variable in jedem Zyklus selbst, überschreibt es den Bedienwert im nächsten
  Zyklus. Bedienvariablen gehören deshalb in `VAR`/`VAR_GLOBAL` und werden vom Programm nur gelesen.
  Dafür eignen sich auch `VDI3814.AV`/`BV`/`MV` (`.Value` ist beschreibbar und wird begrenzt).
- Eingangsvariablen (`%I`, Sensoren) überschreibt das Eingangsabbild jeden Zyklus. Sie lassen sich nur
  anzeigen (zum Testen: Forcen in der IDE).
- Ganzzahlen werden gerundet und auf den Wertebereich des Typs begrenzt, BOOL ist „≠ 0“, TIME in ms.
- Auf der FCU wird abgewiesen, solange ein anderer Browser debuggt (Debug-Lease, Hinweis „ein anderer
  Browser debuggt“). Ohne geladenes Programm wird nichts geschrieben.
- Die Steuerquelle (Feldbus/IEC) spielt keine Rolle: Geschrieben werden SPS-Variablen, keine Register.

**Lesen:** im Simulator jeden Zyklus, auf der FCU über das Watch-Abonnement (höchstens 10 Hz, nur die
Variablen der sichtbaren Elemente und der Linienschreiber).

## Elemente

| Element | Variable | Verhalten |
|---|---|---|
| **Taster** | BOOL | Drücken schreibt TRUE, Loslassen FALSE (*invertiert*: umgekehrt). Gedrückt bleibt er, solange der Zeiger gedrückt ist, auch außerhalb des Elements (Pointer-Capture). Loslassen, Abbruch (Touch) und Verlust des Zeigers schreiben FALSE. |
| **Schalter** | BOOL | Jeder Klick schreibt den **Gegenwert des gelesenen** Zustands. Zwei Bediener, die gleichzeitig klicken, schalten also nicht zweimal um. |
| **Drehpotentiometer** | Zahl | Ziehen nach oben/rechts erhöht (200 px = ganzer Bereich, mit Umschalt 1000 px = fein), Mausrad und Pfeiltasten um eine *Schrittweite* (Bild ↑/↓: zehn). Werte im Raster der Schrittweite, begrenzt auf Min..Max. |
| **Schiebepotentiometer** | Zahl | Wie das Drehpotentiometer, der Wert folgt der Zeigerposition. Waagerecht, wenn das Element breiter als hoch ist, sonst senkrecht. |
| **Digitalanzeige** | jede | Zahl mit *Nachkommastellen* und *Einheit*, BOOL als EIN/AUS, TIME als Dauer, Aufzählungen mit Namen. Ohne Wert „----“. |
| **Analoganzeige** | Zahl | Zeigerinstrument über 240°, Skala Min..Max, Warn- und Alarmbereich farbig (ab *Warnung*/*Alarm* bis Max). Werte außerhalb stehen am Anschlag, die Zahl zeigt den echten Wert. |
| **Balkenanzeige** | Zahl | Füllstand Min..Max (senkrecht, wenn höher als breit); Farbe wechselt ab Warnung/Alarm. |
| **Linienschreiber** | bis 4 Zahlen/BOOL | Kurven über den *Zeitbereich* (Sekunden), Skala Min..Max (fest, keine Autoskalierung). BOOL-Kurven als Stufen im unteren Fünftel (0 → 5 %, 1 → 25 % der Höhe). Legende mit aktuellem Wert. |
| **LED** | BOOL | Rot, Grün, Gelb, Blau, Weiß, Orange; *blinkt wenn an* (1 Hz); *invertiert*. |
| **Meldung** | BOOL | Text und Meldeklasse (1–8 wie `VDI3814.NC`). Leuchtet in der Farbe der Klasse, solange die Meldung ansteht; blinkt, solange sie nicht quittiert ist. Ein Klick in der Laufzeit quittiert. |
| **Text** | – | Beschriftung mit Schriftgröße und Farbe. |

Bedienelemente mit *nur anzeigen* schreiben nicht. Drehpotentiometer und Schieber sind zur Laufzeit per
Tab erreichbar und dann mit den Pfeiltasten bedienbar.

Während ein Drehpotentiometer oder Schieber bedient wird, zeigt es den eingestellten Wert (nicht den
gelesenen), bis 1,5 s nach dem Loslassen. Geschrieben wird beim Ziehen höchstens alle 100 ms und beim
Loslassen der Endwert. Auf der FCU kommen also nicht alle Zwischenwerte an, der Endwert immer.

### Grenzfälle, die man kennen muss

- **Taster und Verbindungsabbruch:** Bricht die Verbindung zur FCU ab, während ein Taster gedrückt ist, kommt
  das FALSE nicht an und die Variable bleibt TRUE. Für Tippbetrieb (Bewegung, solange gedrückt) muss das
  Programm deshalb selbst überwachen, z. B. Tippen nur mit einer Zeitbegrenzung (`TP`) oder einem
  Lebenszeichen. Sicherheitsfunktionen (NOT-HALT) gehören nie auf eine Web-Oberfläche.
- **Ein Taster, zwei Bediener:** Die Variable ist TRUE, solange einer drückt, aber das Loslassen des
  anderen schreibt FALSE. Taster sind für einen Bediener gedacht.
- **Ein Wert außerhalb Min..Max** (vom Programm gesetzt) wird von Drehpotentiometer und Schieber am Rand
  angezeigt. Erst eine Bedienung schreibt einen Wert im Bereich.
- **Min > Max:** Die Skala läuft trotzdem von Min (links/unten, Anfang des Bogens) nach Max. So lässt sich eine
  fallende Skala darstellen; Ziehen nach oben verkleinert dann den Wert.

## Editor

- **Seiten** als Reiter; ＋ legt eine an. Name, Größe (Pixel; zur Laufzeit auf das Fenster skaliert, das
  Seitenverhältnis bleibt), Reihenfolge und Löschen im Eigenschaftenfeld, wenn nichts ausgewählt ist. Dort
  steht auch die Prüfung: Elemente ohne Variable, Variablen, die das Programm nicht kennt.
- **Elemente** aus der Palette auf die Seite ziehen oder anklicken (dann oben links). Raster 10 px.
- **Auswählen:** Klick, Umschalt/Strg+Klick (hinzu/weg), Rahmen aufziehen, Strg+A; Esc hebt auf.
- **Verschieben:** Maus oder Pfeiltasten (Umschalt: 50 px); **Größe:** Griff unten rechts (bei einem
  ausgewählten Element) oder die Felder x/y/b/h; nie über den Seitenrand.
- Entf löscht, Strg+D dupliziert, ⬆/⬇ nach vorn/hinten, Strg+Z/Strg+Y rückgängig/wiederholen (50 Schritte).
- **▶ Bedienen** schaltet die Seite im Editor in den Laufzeitmodus mit dem Gerät der IDE (Simulator oder
  FCU). So lässt sich die Seite gegen das laufende Programm ausprobieren, ohne die Seite zu wechseln.

## Laufzeit

Der Arbeitsbereich *Bedienen & Beobachten*; seine Befehle stehen im Ribbon: Reiter **Bedienen** (die Seiten,
Meldeliste ein/aus, Alle quittieren, Präsentation = nur die Seite im Vollbild) und **Verbindung** (Gerät,
Projektquelle, Neu laden, Im Simulator starten).

- **Gerät:** Simulator dieses Browsers oder FCU. Steht im Simulator kein Programm, bietet ein Hinweis über
  der Seite „▶ Projekt im Simulator starten“ an.
- **Projekt:** das dieses Browsers (der aktuelle Stand der IDE) oder das auf der SD-Karte
  der FCU (beim letzten „Laden“ gespeichert). So braucht ein Bedien-PC keine IDE und kein Projekt: Er holt
  es von der FCU.
- **Meldeliste:** alle Meldungselemente aller Seiten (auch nicht angezeigter). Eine Meldung kommt, wenn ihre
  Variable TRUE wird, und bleibt in der Liste, solange sie ansteht **oder** nicht quittiert ist (Zustände
  wie `VDI3814.NC`). Sortierung: unquittierte zuerst, dann nach Klasse, dann neueste zuerst. Quittieren
  einzeln, „Alle quittieren“ oder ein Klick auf das Element. Mit einer *Quittiervariable* schreibt die
  Laufzeit beim Quittieren TRUE und 300 ms später FALSE. Das Programm sieht eine steigende Flanke
  (z. B. `NC.Ack`), solange sein Zyklus kürzer als 300 ms ist. Darunter steht der Verlauf (kommt, geht,
  quittiert; die letzten 200 Ereignisse).
- **Linienschreiber** zeichnen auf, solange die Laufzeit oder die SPS-Seite sichtbar ist, mit 10 Hz und
  höchstens 2000 Punkten je Element, auch für nicht angezeigte Seiten.
- Gerätezustand, unquittierte Meldungen und „Programm ≠ Projekt“ stehen in der Statusleiste.

**Management-Grenzen (bewusst, Stand dieser Umsetzung):** Meldeliste, Quittierzustand und Verläufe leben
im **Browser**. Zwei Bediener-PCs haben zwei Meldelisten, eine Quittierung an einem PC wirkt am anderen
nicht (außer über die Quittiervariable im Programm). Nach dem Neuladen der Seite sind Verläufe und
Meldeliste leer. Eine Meldung, die kam und ging, während keine Laufzeit offen war, fehlt. Für eine
gemeinsame, dauerhafte Meldeliste müssen die Meldungen in der SPS selbst gespeichert werden (`VDI3814.NC`
mit Quittierung über die Quittiervariable; `NC.State` zeigt dann allen Bedienern denselben Zustand).
Archivierung auf der SD-Karte gibt es noch nicht.
