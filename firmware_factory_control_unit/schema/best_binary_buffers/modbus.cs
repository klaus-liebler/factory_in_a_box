using BestBinaryBuffers;

namespace modbus;

/// <summary>Snapshot ALLER Holding- und Input-Register, binaer -- Nachfolger des frueheren
/// Ad-hoc-REST-Endpunkts GET /api/registers der HAL-Firmware, jetzt ueber denselben
/// WebSocket-Kanal wie alles andere (s. Projektentscheidung "einmalig SPA, dann nur noch binaere
/// WebSockets"). Wird von web/src/apps/modbus-register-app.ts periodisch angefragt.</summary>
[BinaryMessage(MessageKind.Request)]
public class GetRegistersRequest
{
}

[BinaryMessage(MessageKind.Response)]
public class RegistersMessage
{
    // Bounds >= register_map::holding_count / input_count (generated/register_map.hpp), with room
    // for new registers. Too small, the firmware could not encode the answer and sent none (the
    // register console timed out): src/web.cpp checks it at compile time.
    [BinaryMaxItemCount(128)] public ushort[] holding = System.Array.Empty<ushort>();
    [BinaryMaxItemCount(320)] public ushort[] input = System.Array.Empty<ushort>();
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
