# WebSocket-Protokoll (BestBinaryBuffers)

Die Protokoll-Definition liegt jetzt in C#-Schema-Dateien unter `best_binary_buffers_schema/` und
wird mit `BestBinaryBuffers` nach `Core/generated/ws_protocol.hh` und
`web/generated/ws-protocol.ts` kompiliert.

## Phase

```bash
dotnet run --project builder -- GenerateBestBinaryBufferFiles
```

Optional mehrere Quellen anhaengen (wiederholbar):

```bash
dotnet run --project builder -- GenerateBestBinaryBufferFiles -- --best-binary-buffer-schema-path best_binary_buffers_schema
```

Ohne `--best-binary-buffer-schema-path` wird nur `best_binary_buffers_schema/` genutzt.

## IDs

Stabile IDs werden in `best_binary_buffers_schema/ids.txt` gehalten.
Datei nicht manuell umnummerieren.

## Schema-Syntax

Verwendete Attribute und Wire-Regeln kommen aus:
- `C:\repos\dotnet_libs\best_binary_buffers\README.md`

Kurzfassung:
- `[BinaryType]` fuer Typen
- `[BinaryMessage(MessageKind.Event|Request|Response)]` fuer Nachrichten
- `[BinaryUnion]` fuer polymorphe Typfamilien
- Little-endian, string nullterminiert, Array-Count als `ushort`
