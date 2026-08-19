using UniversalRegisterAccess;

[ModbusRegion("Farbsensor")]
public class Color
{
    /// <summary>Farbsensor TCS34725, Clear-Kanal</summary>
    [ModbusRegister(Access.ReadOnly, I2cBus = "I2C_1", I2cIrqPin = "PH0"), ModbusMapping(Address = 160)]
    public ushort ColorClear;

    /// <summary>Farbsensor TCS34725, Rot-Kanal</summary>
    [ModbusRegister(Access.ReadOnly, I2cBus = "I2C_1", I2cIrqPin = "PH0"), ModbusMapping(Address = 161)]
    public ushort ColorRed;

    /// <summary>Farbsensor TCS34725, Gruen-Kanal</summary>
    [ModbusRegister(Access.ReadOnly, I2cBus = "I2C_1", I2cIrqPin = "PH0"), ModbusMapping(Address = 162)]
    public ushort ColorGreen;

    /// <summary>Farbsensor TCS34725, Blau-Kanal</summary>
    [ModbusRegister(Access.ReadOnly, I2cBus = "I2C_1", I2cIrqPin = "PH0"), ModbusMapping(Address = 163)]
    public ushort ColorBlue;
}
