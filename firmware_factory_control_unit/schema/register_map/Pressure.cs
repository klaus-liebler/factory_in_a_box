using UniversalRegisterAccess;

[ModbusRegion("Drucksensor")]
public class Pressure
{
    /// <summary>Analoger Drucksensor, Rohwert (ADC1 CH10)</summary>
    [ModbusRegister(Access.ReadOnly, Gpio = "PA4"), ModbusMapping(Address = 200)]
    public ushort PressureRaw;
}
