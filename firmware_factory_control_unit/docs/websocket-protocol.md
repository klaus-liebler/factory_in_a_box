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
