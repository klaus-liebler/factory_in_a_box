# WebSocket-Protokoll (BestBinaryBuffers)

Die Protokoll-Definition liegt jetzt in C#-Schema-Dateien unter `schema/best_binary_buffers/` und
wird mit `BestBinaryBuffers` nach `generated/ws_protocol.hh` und
`web/generated/ws-protocol.ts` kompiliert.

## Phase

```bash
dotnet run --project builder -- GenerateBestBinaryBufferFiles
```

Optional mehrere Quellen anhaengen (wiederholbar):

```bash
dotnet run --project builder -- GenerateBestBinaryBufferFiles -- --best-binary-buffer-schema-path schema/best_binary_buffers
```

Ohne `--best-binary-buffer-schema-path` wird nur `schema/best_binary_buffers/` genutzt.

## IDs

Stabile IDs werden in `schema/best_binary_buffers/ids.txt` gehalten.
Datei nicht manuell umnummerieren.

## Schema-Syntax

Verwendete Attribute und Wire-Regeln kommen aus:
- `C:\repos\dotnet_libs\best_binary_buffers\README.md`

Kurzfassung:
- `[BinaryType]` fuer Typen
- `[BinaryMessage(MessageKind.Event|Request|Response)]` fuer Nachrichten
- `[BinaryUnion]` fuer polymorphe Typfamilien
- Little-endian, string nullterminiert, Array-Count als `ushort`

## Rahmen: 8-Byte-Kopf

| Feld | Typ | Bedeutung |
|---|---|---|
| namespaceId | u16 | Namespace der Nachricht (`ids.txt`) |
| messageTypeId | u16 | Nachricht im Namespace |
| correlationId | u16 | vom Browser je Anfrage vergeben (1..65535), in der Antwort unveraendert zurueck; 0 = keine Zuordnung (Events, Pushes, Polls per `sendBinary()`) |
| flags | u16 | Bit 0 `FLAG_ERROR`: Fehlerantwort (s.u.), die anderen Bits 0 |

Die Zuordnung Anfrage -> Antwort steht damit im Kopf, nicht in den Nutzdaten: `wsRequest()`
(`web/src/core/ws-client.ts`) wartet nur auf die correlationId.

**Fehlerantwort:** Kann das Geraet eine Anfrage nicht bedienen, antwortet es statt mit Schweigen
(frueher: Timeout nach 3 s) mit einem Rahmen mit `FLAG_ERROR`, Namespace/Typ der Anfrage, ihrer
correlationId und dem Inhalt `code: u16` + Text (nullterminiert). `wsRequest()` verwirft dann sofort
mit `WsErrorReply`. Codes (`ErrorCode`): 1 UnknownMessage (Namespace/Typ unbekannt), 2 Malformed
(Rahmen nicht dekodierbar oder Werte ausserhalb der Grenzen), 3 Busy (Warteschlange voll), 4 Failed;
ab 0x100 anwendungseigene. Ohne correlationId (0) gibt es keine Fehlerantwort.

**Firmware:** Kein Handler fasst den Kopf an. `on_message()` (`src/web.cpp`) liest ihn einmal und
gibt den Handlern einen `WsProtocol::Responder` (generiert): `reply.Send(X::Encode, payload)` traegt
die correlationId der Anfrage ein, `reply.Error(code, text)` antwortet mit Fehler.
`WsProtocol::Dispatch(reply, frame, X::Decode, handler)` dekodiert und ruft den Handler, oder
antwortet `Malformed`. Module, die im I/O- oder SPS-Thread antworten, holen sich ihren Responder mit
`web::responder_from_io()` (sendet ueber die Outbox).

Test: `tools/test-plc-renode.py` (Abschnitt "protocol:") prueft die Fehlerantworten Ende-zu-Ende.
