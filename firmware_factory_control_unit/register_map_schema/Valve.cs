using UniversalRegisterAccess;

// Holding-Adressen 0..19 bleiben absichtlich unbelegt (Reserve, wie im alten register-map.json
// "RESERVED_CONFIG") -- entsteht automatisch durch das Address=20-Pin unten, keine leere Region
// noetig.
[ModbusRegion("Pneumatikventile")]
public class Valve
{
    /// <summary>Pneumatikventil 1</summary>
    [ModbusRegister(Access.ReadWrite, Gpio = "PD10", Display = "bool"), ModbusMapping(Address = 20)]
    public ushort Valve1;

    /// <summary>Pneumatikventil 2</summary>
    [ModbusRegister(Access.ReadWrite, Gpio = "PD5", Display = "bool"), ModbusMapping(Address = 21)]
    public ushort Valve2;

    /// <summary>Pneumatikventil 3</summary>
    [ModbusRegister(Access.ReadWrite, Gpio = "PD11", Display = "bool"), ModbusMapping(Address = 22)]
    public ushort Valve3;
}
