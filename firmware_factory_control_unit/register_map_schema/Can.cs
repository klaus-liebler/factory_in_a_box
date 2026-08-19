using UniversalRegisterAccess;

[ModbusRegion("CAN-Bus")]
public class Can
{
    /// <summary>Gesendete CAN-Frames</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 50)]
    public uint CanTxCount;

    /// <summary>Empfangene CAN-Frames</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 52)]
    public uint CanRxCount;

    /// <summary>CAN-Fehlerzaehler</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 54)]
    public uint CanErrorCount;

    /// <summary>Letzter CAN-Fehlercode</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 56)]
    public ushort CanLastError;

    /// <summary>CAN-Bus-Zustand</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 57)]
    public ushort CanBusState;
}
