using UniversalRegisterAccess;

[ModbusRegion("Stromversorgung")]
public class Power
{
    /// <summary>Busspannung (INA226, 2 mOhm Shunt)</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "mV", I2cBus = "I2C_4"), ModbusMapping(Address = 110)]
    public ushort PwrBusVoltageMv;

    /// <summary>Shunt-Spannung (INA226, 2 mOhm Shunt)</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "uV", I2cBus = "I2C_4"), ModbusMapping(Address = 111)]
    public short PwrShuntVoltageUv;

    /// <summary>Stromaufnahme (INA226, 2 mOhm Shunt)</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "mA", I2cBus = "I2C_4"), ModbusMapping(Address = 112)]
    public short PwrCurrentMa;

    /// <summary>Leistungsaufnahme (INA226)</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "mW", I2cBus = "I2C_4"), ModbusMapping(Address = 113)]
    public ushort PwrPowerMw;

    /// <summary>Aktiv ausgehandelte USB-PD-Spannung, PDSink::activeVoltage</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "mV"), ModbusMapping(Address = 114)]
    public ushort PwrPdVoltageMv;

    /// <summary>Aktiv ausgehandelter USB-PD-Maximalstrom, PDSink::activeCurrent</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "mA"), ModbusMapping(Address = 115)]
    public ushort PwrPdCurrentMa;

    /// <summary>0 = verbunden und Spannung eingestellt, 1 = verbunden aber Spannung nicht
    /// einstellbar, 2 = kein USB-PD-Netzteil erkannt</summary>
    [ModbusRegister(Access.ReadOnly, Display = "status"), ModbusMapping(Address = 116)]
    public ushort PwrPdStatus;

    /// <summary>0 = ok, ungleich 0 = Fehler (aktuell: 1 = INA226 nicht erkannt/Init
    /// fehlgeschlagen)</summary>
    [ModbusRegister(Access.ReadOnly, Display = "status", I2cBus = "I2C_4"), ModbusMapping(Address = 117)]
    public ushort PwrStatus;
}
