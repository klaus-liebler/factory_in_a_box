using UniversalRegisterAccess;

[ModbusRegion("Lichtschranken")]
public class Lightbarrier
{
    /// <summary>Lichtschranke 1 (NPN, aktiv=LOW)</summary>
    [ModbusRegister(Access.ReadOnly, Gpio = "Rev1 PC6, Rev2 PC7", Display = "bool"), ModbusMapping(Address = 180)]
    public ushort Lightbarrier1;

    /// <summary>Lichtschranke 2 (NPN, aktiv=LOW)</summary>
    [ModbusRegister(Access.ReadOnly, Gpio = "Rev1 PC7, Rev2 PC8", Display = "bool"), ModbusMapping(Address = 181)]
    public ushort Lightbarrier2;

    /// <summary>Lichtschranke 3 (NPN, aktiv=LOW) -- nur Hardware-Revision 1</summary>
    [ModbusRegister(Access.ReadOnly, Gpio = "Rev1 PC8", Display = "bool"), ModbusMapping(Address = 182)]
    public ushort Lightbarrier3;
}
