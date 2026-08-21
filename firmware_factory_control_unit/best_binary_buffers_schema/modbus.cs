using BestBinaryBuffers;

namespace modbus;

/// <summary>Snapshot ALLER Holding- und Input-Register, binaer -- Nachfolger des frueheren
/// Ad-hoc-REST-Endpunkts GET /api/registers (Core/Src/webserver.cpp), jetzt ueber denselben
/// WebSocket-Kanal wie alles andere (s. Projektentscheidung "einmalig SPA, dann nur noch binaere
/// WebSockets"). Wird von web/src/apps/modbus-register-app.ts periodisch angefragt.</summary>
[BinaryMessage(MessageKind.Request)]
public class GetRegistersRequest
{
}

[BinaryMessage(MessageKind.Response)]
public class RegistersMessage
{
    // Bounds = ModbusRegisters::HOLDING_REGISTER_MAX_INDEX+1 / INPUT_REGISTER_MAX_INDEX+1
    // (Core/generated/modbus_registers_generated.hh, aus register_map_schema/*.cs generiert) --
    // bei Wachstum der Registerkarte hier nachziehen.
    [BinaryMaxItemCount(111)] public ushort[] holding = System.Array.Empty<ushort>();
    [BinaryMaxItemCount(258)] public ushort[] input = System.Array.Empty<ushort>();
}

/// <summary>Nachfolger von GET /api/write-holding?address=&value=.</summary>
[BinaryMessage(MessageKind.Request)]
public class WriteHoldingRequest
{
    public ushort address;
    public ushort value;
}

[BinaryMessage(MessageKind.Response)]
public class WriteHoldingResponse
{
    public bool success;
}
