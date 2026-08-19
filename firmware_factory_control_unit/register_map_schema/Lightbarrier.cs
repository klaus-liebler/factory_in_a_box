using UniversalRegisterAccess;

[ModbusRegion("Lichtschranken")]
public class Lightbarrier
{
    /// <summary>Lichtschranke 1 (NPN, aktiv=LOW)</summary>
    [ModbusRegister(Access.ReadOnly, Gpio = "PC6", Display = "bool"), ModbusMapping(Address = 180)]
    public ushort Lightbarrier1;

    /// <summary>Lichtschranke 2 (NPN, aktiv=LOW)</summary>
    [ModbusRegister(Access.ReadOnly, Gpio = "PC7", Display = "bool"), ModbusMapping(Address = 181)]
    public ushort Lightbarrier2;

    /// <summary>Lichtschranke 3 (NPN, aktiv=LOW) -- nur auf realem Board verdrahtet</summary>
    [ModbusRegister(Access.ReadOnly, Gpio = "PC8", Display = "bool"), ModbusMapping(Address = 182)]
    public ushort Lightbarrier3;
}
