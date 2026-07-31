# WebSocket-Nachrichtenprotokoll

Binaeres Nachrichtenprotokoll fuer die WebSocket-Verbindung(en) zwischen Browser und Firmware
(`wss://<host>/ws`, s. `Core/Src/http_websocket_server.hpp`/`webserver.cpp`). Jede WebSocket-
Binaerframe traegt genau eine Nachricht -- die Framegrenze der WebSocket-Schicht selbst dient als
aeussere Nachrichtengrenze.

Ziel: HTTP GET dient nur noch dem einmaligen Laden der Single-Page-Anwendung (`/`) sowie einer
optionalen JSON-REST-API unter `/api/*` fuer externe Clients. Der laufende Datenaustausch zwischen
Browser und Firmware (Live-Werte, Befehle, Logs, ...) laeuft ausschliesslich ueber diese
WebSocket-Nachrichten.

## 1. Quelle der Wahrheit

`ws-protocol/*.json` (Repo-Root) ist das von Hand gepflegte Schema -- Dateiname und -aufteilung
sind frei waehlbar (z.B. `ws-protocol/system.json`), **eine Datei entspricht NICHT zwingend genau
einem Namespace**: Namespaces ergeben sich aus dem Namen jeder einzelnen Deklaration (s. Abschnitt
8.0), nicht aus der Datei. Neue Nachrichten/Namespaces werden einfach in eine bestehende oder neue
`*.json`-Datei in diesem Verzeichnis eingetragen. Die builder-Phase
`ReadWebSocketProtocolAndGenerateFiles` (`builder/Phases/ReadWebSocketProtocol.cs`) liest
standardmaessig ALLE Dateien in diesem Verzeichnis und generiert daraus zwei zusammengefasste
Ausgabedateien:

- `Core/generated/ws_protocol.hh` -- C++-Namespaces/Enums/Payload-Structs/Encode-Decode-Funktionen
- `web/generated/ws-protocol.ts` -- gleichwertige TypeScript-Typen/Encode-Decode-Funktionen

Aenderungen an Nachrichten (neue Felder, neue Nachrichtentypen, neue Namespace-Dateien) gehoeren
ausschliesslich in `ws-protocol/*.json`, danach:

```
dotnet run --project builder -- ReadWebSocketProtocolAndGenerateFiles
```

(Teil der Gesamt-Pipelines `pipeline:lazy`/`pipeline:forced`/`pipeline:flash`, s.
docs/build-process.md.)

**Mehrere Quellverzeichnisse/-dateien zusammenfassen:** `--ws-protocol-path` (wiederholbar) haengt
weitere Verzeichnisse (alle direkt darin liegenden `*.json`, nicht rekursiv) oder einzelne
`.json`-Dateien an -- alle zusammen werden zu DEMSELBEN `ws_protocol.hh`/`ws-protocol.ts`
generiert. Relative Pfade werden gegen die Repo-Wurzel aufgeloest. Nuetzlich, wenn z.B. eine
gemeinsam genutzte `stm32_libs`-Bibliothek ihre eigene Namespace-Datei mitbringt:

```
dotnet run --project builder -- ReadWebSocketProtocolAndGenerateFiles \
  -- --ws-protocol-path ws-protocol --ws-protocol-path ../stm32_libs/modbus/ws-protocol
```

Ohne `--ws-protocol-path` bleibt es beim bisherigen Alleinverhalten (nur `ws-protocol/` im
Repo-Root). Message-/Enum-/Struct-/Class-Namen muessen ueber ALLE eingelesenen Dateien (aus allen
Quellen zusammen) eindeutig sein, s. Abschnitt 3 und 8.0.

## 2. Wire-Format

Jede Nachricht beginnt mit einem 4-Byte-Kopf (alle Mehrbyte-Felder Little-Endian, wie das
bestehende `/api/registers`-Binaerformat):

| Offset | Bytes | Feld | Beschreibung |
|---|---|---|---|
| 0 | 2 | `namespaceId` | uint16, identifiziert die fachliche Gruppe (z.B. `system` = 1) |
| 2 | 2 | `messageTypeId` | uint16, identifiziert die Nachricht innerhalb ihres Namespace |

Je nach **Kind** der Nachricht (s. Abschnitt 4) folgt ab Offset 4 ein weiteres Kopf-Feld:

- **Event**: kein weiteres Kopf-Feld -- direkt ab Offset 4 folgt der Payload.
- **Request** / **Response**: zusaetzlich ab Offset 4 ein 2-Byte `requestId` (uint16, vom Client
  frei vergeben, in der zugehoerigen Response unveraendert zurueckgegeben) -- Payload beginnt ab
  Offset 6. `requestId` wird NICHT in `ws-protocol/*.json` deklariert, sondern vom Generator
  automatisch als erstes Feld eingefuegt (s. `EffectiveFields()` in `ReadWebSocketProtocol.cs`).

**Payload:** eine Folge von Feldern in der in `ws-protocol/*.json` deklarierten Reihenfolge. Jedes
Feld ist entweder fest lang (Zahl/bool/Enum, Groesse aus dem Typ bekannt) oder selbstbeschreibend
variabel lang (`string`/`array`, s. Abschnitt 5) -- **beliebig viele** variabel lange Felder sind
an **beliebiger Position** erlaubt, nicht mehr nur eines am Ende. En- und Decodierung laufen
deshalb durchgehend Cursor-basiert (eine laufende Byte-Position, die nach jedem Feld
weiterrueckt), nicht ueber vorausberechnete feste Offsets.

## 3. Datei-/Namespace-/Nachrichten-Struktur

Eine `ws-protocol/*.json`-Datei (Beispiel):

```json
{
  "$comment": "optional, wird ignoriert -- Freitext-Dokumentation wie bei register-map.json",
  "enums": [ { "name": "ns.EnumName", "size": "u8|u16|u32", "description": "optional", "values": [ { "name": "...", "value": 0 } ] } ],
  "structs": [ { "name": "ns.StructName", "description": "optional", "fields": [ ... ] } ],
  "classes": [ { "name": "ns.ClassName", "description": "optional", "fields": [ ... ] } ],
  "messages": [ { "name": "ns.MessageName", "kind": "event|request|response", "description": "optional", "fields": [ ... ] } ]
}
```

Alle vier Listen (`enums`/`structs`/`classes`/`messages`) sind optional und koennen in beliebiger
Kombination und beliebig oft auf mehrere Dateien verteilt vorkommen -- massgeblich fuer die
Namespace-Zugehoerigkeit ist ausschliesslich der Punkt im jeweiligen `"name"` (s. Abschnitt 8.0),
nicht die Datei. Weder Namespaces noch Messages/Classes deklarieren eine numerische `"id"` (s.
Abschnitt 8.5).

Der Namespace-Teil von `name` wird 1:1 als C++-/TypeScript-Namespace-Name uebernommen --
**Vorsicht bei der Namenswahl**:
vermeide gaengige Preprocessor-Makronamen (z.B. `DEBUG`, `ERROR`, `TRUE`) als Namespace-,
Nachrichten-, Feld- oder insbesondere Enum-Wert-Namen. Der urspruengliche Versuch, die
`LogLevel`-Enum-Werte schlicht `TRACE`/`DEBUG`/`INFO`/... zu nennen, brach den Build, weil dieses
Projekt fuer Debug-Builds `-DDEBUG` setzt (CMake `CMAKE_BUILD_TYPE=Debug`) -- der Preprocessor
ersetzte `DEBUG` textuell durch `1`, bevor der Compiler die Enum-Deklaration ueberhaupt sieht
(derselbe Fehlerklasse wie das `_C`-Makro aus `common_macros.hh`, s. Kommentar in
`http_websocket_server.hpp`). Deshalb heissen die Werte jetzt `LOG_TRACE`/`LOG_DEBUG`/... (matcht
zugleich die C-Enum-Namen in `log.h`).

Namespace-Namen/-IDs muessen ueber ALLE eingelesenen Dateien eindeutig sein (bei mehreren
`--ws-protocol-path`-Quellen: ueber ALLE Quellen zusammen, s. Abschnitt 1) -- ein Verstoss
bricht `ReadWebSocketProtocolAndGenerateFiles` sofort mit einer klaren Fehlermeldung (statt eines
schwer zuzuordnenden Namenskonflikts im generierten Code).

## 4. Nachrichten-Kinds

| Kind | Richtung | Zweck | Kopf-Zusatzfeld |
|---|---|---|---|
| `event` | Server -> Client (unaufgefordert) | Server pusht Zustandsaenderungen/Daten, ohne dass der Client danach gefragt hat (z.B. Log-Zeilen, spaeter Live-Registerwerte). | keins |
| `request` | Client -> Server | Client fordert eine Aktion/Antwort an. | `requestId` |
| `response` | Server -> Client | Antwort auf genau einen vorherigen `request` (matching per `requestId`). | `requestId` |

`requestId` erlaubt dem Client, mehrere gleichzeitig ausstehende Requests eindeutig ihren
Responses zuzuordnen (nicht zwingend FIFO/synchron). Es gibt aktuell keine Request/Response-
Nachricht -- dieser Mechanismus ist fuer die geplante spaetere Erweiterung (Befehle,
Live-Abfragen) vorbereitet.

## 5. Feldtypen

| Typ | Groesse | C++ | TypeScript |
|---|---|---|---|
| `uint8`/`int8` | 1 Byte | `uint8_t`/`int8_t` | `number` |
| `uint16`/`int16` | 2 Byte, LE | `uint16_t`/`int16_t` | `number` |
| `uint32`/`int32` | 4 Byte, LE | `uint32_t`/`int32_t` | `number` |
| `uint64`/`int64` | 8 Byte, LE | `uint64_t`/`int64_t` | `number` (s. Einschraenkung unten) |
| `float32` | 4 Byte, LE, IEEE 754 | `float` | `number` |
| `bool` | 1 Byte (0/1) | `bool` | `boolean` |
| `EnumU8`/`EnumU16`/`EnumU32` | 1/2/4 Byte, LE | `enum class <enumName> : uint8_t/uint16_t/uint32_t` | `enum <enumName> { ... }` | 
| `string` | 4-Byte-Bytelaenge (uint32 LE) + UTF-8-Bytes | `const char* + size_t` (Zeiger+Laenge, nicht `'\0'`-terminiert) | `string` (UTF-8 de/encodiert per `TextEncoder`/`TextDecoder`) |
| `array` | 4-Byte-Elementanzahl (uint32 LE) + Elemente | s. u. | `<elementName>[]` |

**`float32`** wird bitweise (nicht wertkonvertierend) uebertragen -- C++ per `memcpy` zwischen
`float` und dem 4-Byte-Puffer, TypeScript per `DataView.getFloat32`/`setFloat32`. Eine
Integer-Bitshift-Implementierung wie bei den anderen Zahlentypen wuerde den Wert KONVERTIEREN statt
das Bitmuster zu uebertragen (`(uint32_t)3.14f == 3`, nicht die IEEE-754-Bytes von `3.14f`).

**`uint64`/`int64`** werden auf C++-Seite wie die anderen Ganzzahltypen behandelt (`uint64_t`
Akkumulator statt `uint32_t`, um Trunkierung beim Encode bzw. undefiniertes Verhalten durch einen
&ge;32-Bit-Schift beim Decode zu vermeiden). Auf TypeScript-Seite gibt es dagegen **keinen**
`DataView`-Zugriff, der eine 64-Bit-Zahl direkt als `number` liest/schreibt (nur
`getBigInt64`/`getBigUint64` mit `bigint`) -- deshalb bildet der Generator 64-Bit-Felder bewusst auf
`number` ab und zerlegt/komponiert sie manuell aus zwei 32-Bit-Haelften
(`Math.floor(value / 4294967296)` fuer die obere, `value >>> 0` fuer die untere Haelfte). Das ist
**exakt fuer alle nicht-negativen Werte bis 2^53** (z.B. Unix-Sekunden oder Millisekunden-Zeitstempel
fuer die naechsten paar Millionen Jahre), aber NICHT fuer negative `int64`-Werte oder Werte oberhalb
von 2^53 -- echte beliebige 64-Bit-Praezision wuerde `bigint` erfordern und ist aktuell nicht
implementiert (kein bekannter sensact/factory_in_a_box-Anwendungsfall braucht das).

**Enum-Felder** brauchen zusaetzlich `"enumName"` (Name des generierten Typs) und `"enumValues"`
(Liste von `{"name": ..., "value": ...}`):

```json
{ "name": "level", "type": "EnumU8", "enumName": "LogLevel", "enumValues": [
  { "name": "LOG_TRACE", "value": 0 }, { "name": "LOG_DEBUG", "value": 1 }
] }
```

**`string`-Felder** koennen mehrfach und an beliebiger Position in einer Nachricht vorkommen --
jedes traegt sein eigenes 4-Byte-Laengenpraefix (Byteanzahl der UTF-8-Kodierung, NICHT
Zeichenanzahl).

**`array`-Felder** brauchen zusaetzlich `"elementName"` (Name des generierten Element-Typs) und
`"fields"` (Feldliste des Elements, rekursiv dieselbe Struktur wie Nachrichtenfelder):

```json
{ "name": "entries", "type": "array", "elementName": "Entry", "fields": [
  { "name": "id", "type": "uint16" },
  { "name": "active", "type": "bool" }
] }
```

**Einschraenkung:** Array-Elemente duerfen NUR feste Feldtypen enthalten (Zahlen/bool/Enum) --
kein verschachteltes `string`/`array`. Grund: nur so hat ein Element eine zur Compile-Zeit
bekannte Bytegroesse (`<elementName>_SIZE`), wodurch sich einzelne Elemente ohne fortlaufenden
Verschachtelungs-Cursor direkt indizieren lassen (`DecodeEntryAt()` in C++, `decodeEntry()` +
Offset-Rechnung in TypeScript). `ReadWebSocketProtocol.cs` lehnt ein verschachteltes
`string`/`array`-Element beim Generieren mit einer klaren Fehlermeldung ab, statt fehlerhaften
Code zu erzeugen. Verschachtelte variable Elemente liessen sich bei Bedarf spaeter nachruesten
(braeuchten dann aber selbst einen Cursor pro Element statt einer festen `_SIZE`-Konstante).

Array-**Encode** in C++ erwartet die Elemente bereits fertig serialisiert im Payload
(`entriesData`/`entriesCount`, roh, s. generierten Code) -- die Anwendung ruft dafuer erst
`EncodeEntry()` je Element in einen eigenen Puffer auf, bevor sie den Gesamt-`Encode()` der
Nachricht aufruft. In TypeScript ist das natuerlicher: `payload.entries: Entry[]`, `encode()`
serialisiert jedes Element automatisch.

## 6. Aktuell definierte Namespaces/Nachrichten

### `system` (namespaceId = 1, `ws-protocol/system.json`)

| Nachricht | typeId | Kind | Felder |
|---|---|---|---|
| `LogMessage` | 1 | event | `level: EnumU8 LogLevel`, `timestampMs: uint32`, `text: string` |

`LogMessage` wird von `Core/Src/ws_log_bridge.cpp` fuer JEDE Logger-Ausgabe (`log_info()` etc., s.
`stm32_libs/common_stm32/log.h`) erzeugt und per `Http::WebServer::Broadcast()` an alle aktuell
verbundenen WebSocket-Clients gesendet. `web/src/ws-client.ts` bildet `level` 1:1 auf die
passende `console.*()`-Funktion ab (`LOG_WARN`->`console.warn`, `LOG_ERROR`/`LOG_FATAL`->
`console.error`, `LOG_INFO`->`console.info`, sonst `console.debug`) -- **wichtig:**
`console.debug()` zaehlt in Chrome DevTools als "Verbose" und ist per Default ausgeblendet;
`console.info()` zaehlt als "Info" und ist per Default sichtbar. Best-Effort/nicht garantiert
zugestellt (s. Kommentar zu `NX_NO_WAIT` in `http_websocket_server.hpp`) -- die UART-Ausgabe
bleibt die primaere, garantierte Log-Senke.

## 7. Generierter Code -- Verwendungsmuster

**C++ (Server, sendend):**

```cpp
#include "generated/ws_protocol.hh"

WsProtocol::system::LogMessage::Payload payload{};
payload.level = WsProtocol::system::LogMessage::LogLevel::LOG_INFO;
payload.timestampMs = HAL_GetTick();
payload.text = "Hallo";
payload.textLength = 5;

uint8_t buffer[256];
size_t len = WsProtocol::system::LogMessage::Encode(payload, buffer, sizeof(buffer));
if (len > 0) {
    App::Instance().web_server.Broadcast(buffer, len);
}
```

**TypeScript (Client, empfangend):** `web/src/ws-client.ts` liest die ersten 4 Bytes jeder
Binaerframe (`namespaceId`/`messageTypeId`), waehlt anhand dieses Paars das passende
`decode()` aus `web/generated/ws-protocol.ts` und ruft den registrierten Handler auf:

```ts
import * as WsProtocol from "../generated/ws-protocol.js";

if (namespaceId === WsProtocol.system.NAMESPACE_ID && messageTypeId === WsProtocol.system.LogMessage.TYPE_ID) {
    const msg = WsProtocol.system.LogMessage.decode(view, 0);
    console.info(msg.text);
}
```

(`decode()` erwartet den KOMPLETTEN Frame inkl. 4-Byte-Kopf plus einen Offset auf dessen Anfang --
im Normalfall `0`, da pro WebSocket-Frame genau eine Nachricht uebertragen wird.)

## 8. Namespaces, Struct/Class/heterogene Arrays/Einzelfelder, geteilte Enums, ID-Zuordnung

Erweiterung des Generators (`ReadWebSocketProtocol.cs`), vor allem fuer die Uebernahme bestehender
Flatbuffers-Schemas (z.B. aus dem sensact-Projekt) noetig: Namespaces sind keine Dateien mehr,
sondern ergeben sich aus dem Namen jeder Deklaration; feste Verbund-Typen (`struct`); Array-
Element-Typen mit Strings (`classes`), sowohl als Liste als auch als Einzelfeld; wiederverwendbare,
benannte Enums; fest wiederholte Skalarfelder (`count`); eine Namen- statt ID-basierte
Schema-Pflege.

### 8.0 Namespaces ergeben sich aus dem Namen, nicht aus der Datei

**Wichtige Aenderung ggue. frueheren Versionen dieses Schemas**: eine `ws-protocol/*.json`-Datei
entspricht **nicht mehr** zwingend genau einem Namespace. Stattdessen traegt JEDE Deklaration
(Message/Enum/Struct/Class) ihren Namespace direkt im eigenen `"name"`:

- Enthaelt der Name einen Punkt (`"wifimanager.AccessPoint"`), ist der Teil vor dem ersten Punkt
  der Namespace-Name, der Rest der lokale Name.
- Enthaelt der Name KEINEN Punkt (`"Foo"`), gehoert die Deklaration zum **NULL-Namespace**
  (Namespace-Name `""`, `NAMESPACE_ID` immer fest `0`, nicht Teil der normalen ID-Vergabe). Im
  generierten Code liegt eine NULL-Namespace-Deklaration ohne umschliessenden `namespace {}`-Block
  direkt auf `WsProtocol`-Ebene (C++) bzw. Modul-Ebene (TS).

Daraus folgt: eine einzelne Datei kann Deklarationen mehrerer Namespaces enthalten, und ein
einzelner Namespace kann sich ueber beliebig viele Dateien erstrecken (auch ueber mehrere
`--ws-protocol-path`-Quellen hinweg). Referenzen (`structRef`/`enumRef`/`classes`) verwenden
denselben Namen wie die Deklaration -- KEINE gesonderte "innerhalb des eigenen Namespace reicht der
unqualifizierte Name"-Abkuerzung: `"AccessPoint"` referenziert immer den NULL-Namespace-Eintrag
`AccessPoint`, niemals implizit `wifimanager.AccessPoint`, selbst wenn die Referenz aus einer
`wifimanager.*`-Nachricht heraus erfolgt.

### 8.1 Structs -- reine, feste Datenverbuende

Eine Datei kann `"structs"` deklarieren -- Verbuende, die ausschliesslich aus festen Feldtypen
bestehen (Zahlen/bool/Enum/EnumRef/weitere Structs, rekursiv, optional mit `"count"` s. Abschnitt
8.6). Ein Struct hat keinen eigenen Kopf/keine eigene ID (wird nie selbst als Nachricht
verschickt), sondern wird ueber `{"type":"struct","structRef":"<Name>"}` als Feld in eine
Message/Class/ein anderes Struct eingebettet:

```json
{
  "structs": [
    { "name": "Mac6", "description": "6-Byte-MAC-Adresse (NULL-Namespace, kein Punkt im Namen)", "fields": [
      { "name": "b", "type": "uint8", "count": 6 }
    ]}
  ],
  "messages": [
    { "name": "systeminfo.ResponseSystemData", "kind": "response", "fields": [
      { "name": "staMac", "type": "struct", "structRef": "Mac6" }
    ]}
  ]
}
```

Generiert wird u.a. `<Name>_SIZE` (Compile-Zeit-Konstante),
`<Name>Encode`/`<Name>Decode` (C++, Cursor-basiert: `size_t <Name>Encode(const <Name>&, uint8_t* dest, size_t pos, size_t dest_size)`,
`bool <Name>Decode(const uint8_t* data, size_t len, size_t& pos, <Name>& out)`) sowie
`encode<Name>Into`/`encode<Name>`/`decode<Name>` (TypeScript).

### 8.2 Classes -- Array-/Einzelfeld-Element-Typen mit Strings, auch gemischt

Eine Datei kann `"classes"` deklarieren -- wie eine Message (Name + Felder), aber ohne Kopf/
eigenstaendigen Nachrichtentyp, und DARF Strings enthalten (im Unterschied zum Struct). Eine Class
darf selbst KEIN Array-Feld enthalten (ein Array darf nur in einer Message stehen, nicht rekursiv
in den Objekten eines Arrays), aber -- als jeweils letztes Feld -- ein weiteres einzelnes
polymorphes Feld (s. 8.2b).

```json
{
  "classes": [
    { "name": "wifimanager.AccessPoint", "fields": [
      { "name": "ssid", "type": "string" }, { "name": "rssi", "type": "int32" }
    ]}
  ],
  "messages": [
    { "name": "wifimanager.ResponseNetworkInformation", "kind": "response", "fields": [
      { "name": "hostname", "type": "string" },
      { "name": "accesspoints", "type": "array", "classes": ["wifimanager.AccessPoint"] }
    ]}
  ]
}
```

Ein Array-Feld referenziert per `"classes": ["<Name>", ...]` eine oder mehrere zulaessige Klassen
(aus beliebigen Namespaces, auch gemischt) -- bei mehr als einer Klasse ist das Array
**heterogen** (jedes Element kann eine andere der gelisteten Klassen sein).

**Wichtige Einschraenkung:** Ein Array-Feld mit `"classes"` MUSS Teil eines zusammenhaengenden
Blocks solcher Felder am ENDE seines umschliessenden Objekts (Message oder Class) sein -- nach dem
ersten `classes`-Array/Einzelfeld duerfen nur noch WEITERE `classes`-Array-/Einzelfelder folgen,
kein Feld eines anderen Typs mehr. Mehrere `classes`-Arrays hintereinander sind ausdruecklich
erlaubt (z.B. `fingerprint.ResponseFingers` mit `scheduleNames` UND `fingers`, je ein eigenes
Array) -- Encode/Decode laufen ohnehin sequenziell durch alle Felder und jedes Element ist
selbstbeschreibend (2-Byte-`classId` + Klassenfelder), es gibt also keinen technischen Grund, mehr
als eines zu verbieten. Grund fuer die Einschraenkung ueberhaupt: Klassen-Elemente koennen Strings
enthalten und sind deshalb unterschiedlich lang -- die exakte Gesamtbytelaenge eines solchen Arrays
ist ohne es vollstaendig zu durchlaufen nicht bekannt. Am Ende (auch mehrfach hintereinander) ist
das unproblematisch; VOR einem Feld eines anderen Typs wuerde es verhindern, den Cursor korrekt
dorthin vorzuruecken, ohne das Array bereits (doppelt) dekodiert zu haben. Der Generator lehnt eine
falsch platzierte `classes`-Deklaration mit einer klaren Fehlermeldung ab.

**C++ (allokationsfrei):** die Nachricht speichert nur `<Feld>Data`/`<Feld>Count`/`<Feld>DataSize`
(vorserialisierte Rohbytes). Zum Senden schreibt man jedes Element ueber eine generierte
`Append<Owner><Feld><Klasse>Element(item, dest, pos, dest_size)`-Hilfsfunktion in einen
Scratch-Puffer und uebergibt Puffer+Anzahl+Bytelaenge; zum Empfangen laeuft man die Elemente ueber
eine generierte `Decode<Owner><Feld>Elements(data, dataSize, count, visitor)`-Funktion ab (Template
mit Callback -- kein `std::variant`, keine Heap-Allokation), die je nach vorangestelltem
2-Byte-`classId` den passenden Klassen-Decoder aufruft und mit dem dekodierten Payload den Visitor
aufruft.

**TypeScript (materialisiert):** `encode()`/`decode()` behandeln das Feld wie ein normales,
vollstaendig typisiertes Array einer Discriminated Union (`{classId, ...Feld1} | {classId, ...Feld2} | ...`) --
kein manuelles Nachbearbeiten noetig.

### 8.2b Einzelnes polymorphes Feld (`"type":"class"`)

Wie 8.2, aber fuer ein einzelnes Feld statt einer Liste -- z.B. "eine von mehreren Schedule-
Varianten", ohne dass es sich um eine Liste handelt:

```json
{ "name": "schedule", "type": "class", "classes": ["scheduler.Predefined", "scheduler.SunRandom"] }
```

Wire-Format identisch zu einem einzelnen Element eines heterogenen Arrays (2-Byte `classId` +
Klassenfelder), aber OHNE Laengen-/Anzahl-Praefix. Unterliegt derselben "muss Teil des
trailing Blocks sein"-Einschraenkung wie 8.2, aus demselben Grund. C++ und TypeScript nutzen
dieselben generierten `Append*Element`/`Decode*Elements`-Funktionen wie das heterogene Array
(im TS-Fall direkt inline dekodiert statt in einer Schleife).

### 8.3 Geteilte, benannte Enums (`"enums"` + `"enumRef"`)

Bisher musste jedes Enum-Feld sein Enum (Name + alle Werte) inline an der Verwendungsstelle
deklarieren (`"type":"EnumU8", "enumName":..., "enumValues":[...]`) -- unpraktisch fuer ein Enum,
das (wie sensacts `ApplicationId`/`CommandType`) an vielen Stellen wiederverwendet wird. Eine Datei
kann daher zusaetzlich `"enums"` deklarieren:

```json
{
  "enums": [
    { "name": "sensact.ApplicationId", "size": "u16", "values": [
      { "name": "NO_APPLICATION", "value": 65535 },
      { "name": "MASTER", "value": 0 }
    ]}
  ],
  "messages": [
    { "name": "sensact.RequestCommand", "kind": "request", "fields": [
      { "name": "id", "type": "enumRef", "enumRef": "sensact.ApplicationId" }
    ]}
  ]
}
```

`"size"` ist `"u8"`/`"u16"`/`"u32"` (analog zu `EnumU8`/`EnumU16`/`EnumU32` bei der inline-Variante).
Ein geteiltes Enum braucht keine eigene ID (nie eigenstaendig auf dem Wire identifiziert, nur der
Typ wird zur Compile-Zeit referenziert) und kann -- wie Structs/Classes -- aus jeder anderen
eingelesenen Datei referenziert werden, unabhaengig von Namespace-/Dateigrenzen (s. 8.5).

### 8.4 Fest wiederholte Skalar-/Enum-/Struct-Felder (`"count"`)

Ein optionales `"count": N` auf einem `Fixed`-, `Enum`-, `enumRef`- oder `struct`-Feld macht daraus
ein C-Array aus `N` gleichartigen Werten, statt `N` einzeln benannte Felder auflisten zu muessen --
z.B. ein 84-Byte-Wochenplan (`OneWeekIn15MinutesData`) als `{"name":"v","type":"uint8","count":84}`
statt 84 einzelner `b0`..`b83`-Felder. Bleibt ein fester Feldtyp (Groesse = Elementgroesse × Count),
also weiterhin auch innerhalb eines Structs oder als homogenes Array-Element verwendbar.

### 8.5 ID-Zuordnung: keine IDs mehr in der JSON, stattdessen `ws-protocol/ids.txt`

Namespaces, Messages und Classes deklarieren **keine** numerische `"id"` mehr in ihrer JSON-Datei
-- sie werden ausschliesslich ueber ihren Namen identifiziert. Enums/Structs brauchen keine ID
(nie eigenstaendig auf dem Wire identifiziert, immer eingebettet/nur zur Compile-Zeit
referenziert).

Beim Generieren vergibt der Generator fuer jeden neu angetroffenen Namen automatisch die naechste
freie ID im jeweiligen Zaehlerkreis (Namespace- und Class-IDs GLOBAL fortlaufend -- eine
Klassenliste kann Klassen aus verschiedenen Namespaces mischen, der 2-Byte-Discriminator muss
dann ohne Namespace-Praefix eindeutig sein; Message-IDs PRO NAMESPACE fortlaufend ab 1, der
NULL-Namespace eingeschlossen) und haelt die Zuordnung dauerhaft in der einfachen Textdatei
`ws-protocol/ids.txt` fest (`<kind> <name> <id>` je Zeile, `#`-Kommentarzeilen erlaubt). Diese
Datei wird **mitversioniert** (nicht gitignored) und bei jedem Lauf nur ergaenzt, nie
umsortiert/umnummeriert -- einmal vergebene IDs bleiben so ueber beliebig viele weitere
Codegenerierungen (auch bei Schema-Erweiterungen) stabil, selbst wenn ein Name spaeter aus dem
Schema entfernt wird (die Zeile bleibt einfach stehen, die ID wird nie wiederverwendet).

### 8.6 Kein `include` noetig -- Zwei-Pass-Verarbeitung

Ein Enum/Struct/eine Class kann aus jeder anderen im selben Lauf eingelesenen Datei referenziert
werden (auch aus einer anderen `--ws-protocol-path`-Quelle, s. Abschnitt 1, und unabhaengig von
Namespace-Grenzen, s. 8.0), ohne dass die referenzierende Datei die deklarierende Datei
"importieren" muesste. Dafuer liest der Generator alle Dateien in zwei Durchgaengen: **Pass 1**
sammelt nur Namen/rohe JSON-Deklarationen aller `enums`/`structs`/`classes` ueber ALLE Dateien
hinweg in globale Registries; **Pass 2** loest dann jede tatsaechliche Feld-Referenz (`enumRef`/
`structRef`/`classes`-Liste) gegen diese Registries auf -- unabhaengig davon, aus welcher Datei
die Referenz kommt oder in welcher Reihenfolge die Dateien eingelesen wurden.
