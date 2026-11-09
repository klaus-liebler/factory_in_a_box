# Kommandos: bauen, testen, Board wechseln

Alle Kommandos laufen im Verzeichnis `firmware_factory_control_unit/`, wenn nichts anderes dasteht.
Die Details des Builders stehen in [build-process.md](build-process.md); hier steht, was man im Alltag
eintippt.

## 1. Die drei Stellschrauben

Ein Firmware-Build wird durch drei Dinge bestimmt. Sie sind unabhängig voneinander.

| Stellschraube | Was sie festlegt | Wo man sie einstellt |
|---|---|---|
| **Preset** | Hardware-Revision (Chip, Pin-Belegung) und Debug/Release | `--preset <Name>` bei `cmake` und beim Builder |
| **Board** | Identität eines einzelnen Geräts: Chip-UID, Hostname, MAC-Adressen, Zertifikate | Builder: ST-Link angeschlossen oder `--board <Board-ID>` |
| **Stand** | Quelltext, eingebettete Web-App, erzeugte Dateien (`generated/`) | Git, Builder-Schritte |

### Presets

| Preset | Platine | Chip | Besonderheit |
|---|---|---|---|
| `Release-Rev1`, `Debug-Rev1` | Revision 1 (vorhandene Hardware) | STM32H563 | USB-PD über UCPD; **keine DIAG-Leitungen** der TMC2209 |
| `Release-Rev3`, `Debug-Rev3` | Revision 3 (wird als nächste gebaut) | STM32H743 | DIAG-Leitungen verdrahtet |
| `Release`, `Debug` | Revision 2 | STM32H563 | nie als Hardware gebaut: wird übersetzt, aber **nicht getestet** |

`Release` ist auf Größe optimiert (`-Os`), `Debug` ohne Optimierung zum Debuggen. Jedes Preset hat
seinen eigenen Ordner `build/<Preset>/`; die fertige Firmware ist
`build/<Preset>/firmware_factory_control_unit.elf`.

### Board

Jedes Gerät hat einen Ordner im Board-Archiv (`BoardsDir` in `builder/appsettings.json`, zurzeit
`OneDrive - HSOS/stm32_boards/<Board-ID>/`). Die Board-ID ist `<letzte 6 Stellen>_<Chip-UID>`, zum
Beispiel `303537_003400343035510834303537` (die Revision-1-Platine, Hostname `factory-box-303537`).
Dort liegen Zertifikate, Schlüssel und der Verlauf der Flash-Vorgänge.

Die Firmware prüft beim Start, ob die Chip-UID zu ihrer eingebauten Identität passt. Eine Firmware,
die für Board A gebaut ist, läuft also nicht auf Board B.

## 2. Der Builder

`builder/` ist ein .NET-Programm, das alles erzeugt, was **vor** dem eigentlichen Übersetzen da sein
muss, und das Flashen übernimmt. `cmake` allein übersetzt nur den C++-Code mit dem, was der Builder
zuletzt hingelegt hat.

| Aufgabe | Builder-Schritt | Ergebnis |
|---|---|---|
| Board erkennen | `PrepareContextWithRealHardware` (ST-Link) oder `PrepareContextWithCommandLineArguments -- --board <ID>` | merkt sich das Board in `build/.last-board-id` |
| Zertifikate | `GenerateCertificatesLazy` (nur wenn sie fehlen), `GenerateCertificatesForced` (immer neu) | im Board-Archiv, signiert von der Root-CA |
| Geräte-Identität | `GenerateDeviceArtifacts` | `generated/device_ids.hh` |
| Registerkarte | `GenerateRegisterAccessFiles` | aus `schema/register_map/*.cs`: C++ und TypeScript |
| WebSocket-Protokoll | `GenerateBestBinaryBufferFiles` | aus `schema/best_binary_buffers/*.cs`: `ws_protocol.hh`, `ws-protocol.ts` |
| Terminal-Kommandos | `GenerateTerminalFiles` | aus `schema/terminal/` |
| Versionsnummer | `ReadGitStatusAndGenerateFiles` | Git-Stand in die Firmware |
| Zertifikate bereitlegen | `CopyGeneratedFilesToBuildDirectory` | `build/assets/*.der` |
| Web-App | `BuildWebApp` | `build/assets/index.html.br` (wird in die Firmware eingebettet) |
| Firmware | `BuildFirmware -- --preset <Preset>` | ruft `cmake` auf |
| Flashen | `FlashFirmware -- --preset <Preset>` | über ST-Link, trägt den Vorgang im Board-Archiv ein |

Aufruf immer in dieser Form; Parameter für den Schritt stehen hinter dem zweiten `--`:

```
dotnet run --project builder -- <Schritt> [-- --preset <Preset>] [--board <Board-ID>]
```

Drei Sammelschritte fassen alles zusammen. Alle drei brauchen ein angeschlossenes Board (ST-Link):

| Sammelschritt | Was er tut |
|---|---|
| `PipelineLazy` | alle Schritte bis zur fertigen Firmware, ohne Flashen |
| `PipelineForced` | wie `PipelineLazy`, erzeugt aber die Zertifikate neu |
| `PipelineFlash` | wie `PipelineLazy`, flasht danach |

Fehlt `--preset`, gilt `Release` (Revision 2). Für die echten Platinen also immer `--preset` angeben.

## 3. Bauen

**Board am ST-Link, alles bauen und flashen (der Normalfall):**

```
dotnet run --project builder -- PipelineFlash -- --preset Release-Rev1
```

**Alles bauen, nicht flashen:**

```
dotnet run --project builder -- PipelineLazy -- --preset Release-Rev1
```

**Ohne angeschlossenes Board bauen** (das Board muss schon einmal geflasht worden sein, also im
Archiv stehen). Das Board einmal wählen, dann die Schritte einzeln:

```
dotnet run --project builder -- PrepareContextWithCommandLineArguments -- --board 303537_003400343035510834303537
dotnet run --project builder -- GenerateDeviceArtifacts
dotnet run --project builder -- ReadGitStatusAndGenerateFiles
dotnet run --project builder -- CopyGeneratedFilesToBuildDirectory
dotnet run --project builder -- BuildWebApp
dotnet run --project builder -- BuildFirmware -- --preset Release-Rev1
```

**Nur den C++-Code neu übersetzen** (nach einer Änderung in `src/` oder `rtoos/`; Web-App und
erzeugte Dateien bleiben, wie sie sind):

```
cmake --build --preset Release-Rev1
cmake --build --preset Release-Rev3
```

Beim ersten Mal für ein Preset vorher einmal `cmake --preset <Preset>` (legt `build/<Preset>/` an).

**Nach einer Änderung an der Web-App** (`web/src/`): erst die Web-App, dann die Firmware, denn die
Firmware bettet die Web-App ein.

```
dotnet run --project builder -- BuildWebApp
cmake --build --preset Release-Rev1
```

**Nach einer Änderung an einem Schema** (`schema/…/*.cs`): den passenden Erzeuger laufen lassen,
dann bauen.

```
dotnet run --project builder -- GenerateBestBinaryBufferFiles     # WebSocket-Protokoll
dotnet run --project builder -- GenerateRegisterAccessFiles       # Registerkarte
```

**Flashen einer schon gebauten Firmware:**

```
dotnet run --project builder -- FlashFirmware -- --preset Release-Rev1
```

## 4. Zwischen Boards und Revisionen umschalten

- **Andere Revision, gleiches Board:** nur ein anderes `--preset`. Die Ordner `build/<Preset>/`
  stehen nebeneinander, nichts muss gelöscht werden.
- **Anderes Board:** Board an den ST-Link hängen und `PipelineFlash` (oder `PipelineLazy`) starten;
  der Builder liest die Chip-UID und nimmt die Identität dieses Boards. Ohne Hardware:
  `PrepareContextWithCommandLineArguments -- --board <ID>` und danach `GenerateDeviceArtifacts` und
  `CopyGeneratedFilesToBuildDirectory`. `generated/` und `build/assets/` gehören immer zu genau
  einem Board, nämlich dem zuletzt gewählten (`build/.last-board-id`).
- **Neues Board (z. B. die erste Revision-3-Platine):** anschließen und
  `dotnet run --project builder -- PipelineFlash -- --preset Release-Rev3`. Der Builder legt den
  Archiv-Ordner an, erzeugt Hostname, MAC-Adressen und Zertifikate und flasht.

Welches Board gerade gilt: `cat build/.last-board-id`. Welche Boards es gibt: die Ordner im
Board-Archiv.

## 5. Testen

Es gibt vier Ebenen. Keine braucht echte Hardware außer der letzten.

### 5.1 Web-App und SPS-Compiler (Sekunden)

```
cd web
npm test                 # alle vitest-Tests (Compiler, VM, IDE, Editoren, Stepper-Modell)
npm run typecheck        # TypeScript der ganzen Web-App
npm run typecheck:plc    # TypeScript des SPS-Teils (strenger)
npx vitest run src/plc/ide/ide.test.ts     # eine einzelne Testdatei
npm run dev              # Web-App im Browser entwickeln (ohne Gerät)
npm run plc:corpus       # Testkorpus und Renode-Testimage der SPS neu erzeugen (nach Compiler-Änderungen)
```

### 5.2 rtoos auf dem PC (Host-Tests, etwa eine halbe Minute)

Treiber, Netzwerk-Stack, OPC UA, SPS-VM, Stepper-Rampen - auf dem PC übersetzt, ohne Emulator.

```
cd rtoos
cmake --preset host                         # einmalig
cmake --build --preset host
ctest --preset host                         # alle (über 800)
ctest --preset host -R "Stepper|Tmc"        # nur Tests, deren Name passt (regulärer Ausdruck)
ctest --preset host -R OpcUa --output-on-failure
```

### 5.3 rtoos im Emulator (Minuten)

Die rtoos-Testprogramme laufen in QEMU und Renode auf dem emulierten Chip.

```
cd rtoos
cmake --preset arm       && cmake --build --preset arm       && ctest --preset arm        # STM32H563 (Revision 1)
cmake --preset arm-h743  && cmake --build --preset arm-h743  && ctest --preset arm-h743   # STM32H743 (Revision 3)
ctest --preset arm -R stepper_renode        # ein einzelner Test
```

Die Netzwerk-Tests brauchen den TAP-Adapter `rtoos-tap` (einmalig, als Administrator:
`rtoos/emulation/renode/setup-tap.ps1`).

### 5.4 Die FCU-Firmware im Emulator (Ende-zu-Ende, einige Minuten)

Die echte Firmware läuft in Renode; die Skripte sprechen sie über HTTPS/WebSocket und Modbus an.
Vorher die Firmware des passenden Presets bauen. Die Skripte nehmen die Chip-UID aus
`generated/device_ids.hh`, also vom zuletzt gewählten Board.

```
python tools/test-plc-renode.py --revision 1     # SPS, Watch, Forcen, Haltepunkte, Schrittmotoren (mit Fahrten)
python tools/test-plc-renode.py --revision 3     # dasselbe auf dem H743 (ohne Fahrten: Renode fehlt dort das Timer-DMA)
python tools/test-missions-sd-renode.py          # RoArm-Missionen auf der SD-Karte (Revision 3)
python tools/test-usb-renode.py                  # USB (Revision 3)
```

| Parameter | Bedeutung |
|---|---|
| `--revision 1\|3` | welche Platine emuliert wird; wählt auch die Firmware (`build/Release-Rev1` bzw. `-Rev3`) |
| `--elf <Datei>` | eine andere Firmware-Datei als die Standard-Datei des Presets |
| `--renode <Pfad>` | Renode an anderer Stelle (Standard `C:/opt/renode/renode.exe`) |

Am Ende steht `RESULT: PASS` oder `RESULT: FAIL (n)`; jede Prüfung ist eine Zeile `[ok]`/`[FAIL]`.

**Die Firmware im Emulator laufen lassen und selbst ausprobieren** (Web-Oberfläche unter
`https://192.168.77.2/`, bis Strg+C):

```
python tools/run-renode-fcu.py --revision 3
python tools/run-renode-fcu.py --revision 1 --quiet        # ohne das Firmware-Log
python tools/run-renode-fcu.py --revision 3 --seconds 300  # endet nach 5 Minuten von selbst
```

### 5.5 Gegen ein laufendes Gerät (echte Platine oder `run-renode-fcu.py`)

```
python tools/test-https-and-websocket.py --host factory-box-303537.local --ca-cert <rootCA.pem.crt>
python tools/test_modbus.py --host factory-box-303537.local
python tools/test_modbus.py --host 192.168.77.2 --set-valve 1 on
python tools/test_opcua.py --host 192.168.77.2 --server-cert <Board-Archiv>/factory-box-303537.pem.crt
```

| Parameter | Bedeutung |
|---|---|
| `--host` | Hostname oder IP des Geräts (Emulator: `192.168.77.2`, über USB: `192.168.173.1`) |
| `--ca-cert` | Root-CA, mit der das HTTPS-Zertifikat des Geräts geprüft wird |
| `--server-cert` | OPC-UA-Zertifikat des Geräts aus dem Board-Archiv (`factory-box-<id>.pem.crt`) |

**OPC UA braucht ein Client-Zertifikat der Root-CA.** Der Server nimmt nur Clients an, deren
Zertifikat von der CA stammt, der die Firmware vertraut. Beim ersten Lauf von `test_opcua.py`
deshalb einmal die CA angeben; das Skript erzeugt sich ein Client-Zertifikat und lässt es von
`openssl` signieren (der CA-Schlüssel wird nur von `openssl` an Ort und Stelle gelesen):

```
python tools/test_opcua.py --host 192.168.77.2 --server-cert <…>/factory-box-303537.pem.crt --ca-cert <…>/rootCA.pem.crt --ca-key <…>/rootCA.pem.key
```

Danach liegt die Test-Identität in `tools/.opcua-test-client/` (nicht in Git) und die beiden
`--ca-…`-Parameter entfallen.

## 6. Typische Abläufe

**Vor einem Commit** (Änderung an Firmware und Web-App):

```
cd web && npm test && npm run typecheck && cd ..
cd rtoos && cmake --build --preset host && ctest --preset host && cd ..
cmake --build --preset Release-Rev1 && cmake --build --preset Release-Rev3
python tools/test-plc-renode.py --revision 1
python tools/test-plc-renode.py --revision 3
```

**Neue Firmware auf die Revision-1-Platine:**

```
dotnet run --project builder -- PipelineFlash -- --preset Release-Rev1
```

**Nur die Web-Oberfläche geändert, schnell ansehen:** `cd web && npm run dev` (ohne Gerät), oder mit
Firmware: `dotnet run --project builder -- BuildWebApp`, `cmake --build --preset Release-Rev3`,
`python tools/run-renode-fcu.py --revision 3`.
