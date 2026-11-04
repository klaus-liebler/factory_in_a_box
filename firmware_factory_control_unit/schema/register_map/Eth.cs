using UniversalRegisterAccess;

[ModbusRegion("Ethernet")]
public class Eth
{
    /// <summary>0 = down, 1 = up</summary>
    [ModbusRegister(Access.ReadOnly, Display = "bool"), ModbusMapping(Address = 80)]
    public ushort EthLinkStatus;

    /// <summary>Verbindungsgeschwindigkeit (0 = kein Link)</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "Mbit/s"), ModbusMapping(Address = 81)]
    public ushort EthLinkSpeed;

    /// <summary>0=half, 1=full</summary>
    [ModbusRegister(Access.ReadOnly, Display = "bool"), ModbusMapping(Address = 82)]
    public ushort EthLinkDuplex;

    /// <summary>Gesendete Ethernet-Frames</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 83)]
    public uint EthTxCount;

    /// <summary>Empfangene Ethernet-Frames</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 85)]
    public uint EthRxCount;

    /// <summary>Ethernet-Fehlerzaehler</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 87)]
    public uint EthErrorCount;

    /// <summary>Letzter Ethernet/PHY-Fehlercode</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 89)]
    public ushort EthLastError;
}
