using UniversalRegisterAccess;

[ModbusRegion("ToF-Sensoren")]
public class Tof
{
    /// <summary>Abstand Sensor 1 (VL53L0X)</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "mm", I2cBus = "I2C_1", I2cIrqPin = "PC15"), ModbusMapping(Address = 140)]
    public ushort Tof1DistanceMm;

    /// <summary>0 = ok, ungleich 0 = Fehler (aktuell: 1 = nicht erkannt/keine gueltige
    /// Messung)</summary>
    [ModbusRegister(Access.ReadOnly, Display = "status", I2cBus = "I2C_1", I2cIrqPin = "PC15"), ModbusMapping(Address = 141)]
    public ushort Tof1Status;

    /// <summary>Abstand Sensor 2 (VL53L0X)</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "mm", I2cBus = "I2C_2", I2cIrqPin = "PH1"), ModbusMapping(Address = 142)]
    public ushort Tof2DistanceMm;

    /// <summary>0 = ok, ungleich 0 = Fehler (aktuell: 1 = nicht erkannt/keine gueltige
    /// Messung)</summary>
    [ModbusRegister(Access.ReadOnly, Display = "status", I2cBus = "I2C_2", I2cIrqPin = "PH1"), ModbusMapping(Address = 143)]
    public ushort Tof2Status;

    /// <summary>Abstand Sensor 3 (VL53L0X)</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "mm", I2cBus = "I2C_4", I2cIrqPin = "PA3"), ModbusMapping(Address = 144)]
    public ushort Tof3DistanceMm;

    /// <summary>0 = ok, ungleich 0 = Fehler (aktuell: 1 = nicht erkannt/keine gueltige
    /// Messung)</summary>
    [ModbusRegister(Access.ReadOnly, Display = "status", I2cBus = "I2C_4", I2cIrqPin = "PA3"), ModbusMapping(Address = 145)]
    public ushort Tof3Status;
}
