using UniversalRegisterAccess;

[ModbusRegion("Digitaleingaenge")]
public class Gpio
{
    /// <summary>GPIO 1 (J30), binaerer Eingang, aktiv=HIGH (interner Pull-down) -- nur Hardware-Revision 3</summary>
    [ModbusRegister(Access.ReadOnly, Gpio = "Rev3 PE2", Display = "bool"), ModbusMapping(Address = 185)]
    public ushort Gpio1;

    /// <summary>GPIO 2 (J31), binaerer Eingang, aktiv=HIGH (interner Pull-down) -- nur Hardware-Revision 3</summary>
    [ModbusRegister(Access.ReadOnly, Gpio = "Rev3 PE3", Display = "bool"), ModbusMapping(Address = 186)]
    public ushort Gpio2;
}
