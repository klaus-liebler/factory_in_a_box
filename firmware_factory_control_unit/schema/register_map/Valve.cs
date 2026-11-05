using UniversalRegisterAccess;

// Holding-Adressen 0..19 bleiben absichtlich unbelegt (Reserve, wie im alten register-map.json
// "RESERVED_CONFIG") -- entsteht automatisch durch das Address=20-Pin unten, keine leere Region
// noetig.
[ModbusRegion("Pneumatikventile")]
public class Valve
{
    /// <summary>Pneumatikventil 1</summary>
    [ModbusRegister(Access.ReadWrite, Gpio = "Rev1 PD10, Rev2 PA8", Display = "bool"), ModbusMapping(Address = 20)]
    public ushort Valve1;

    /// <summary>Pneumatikventil 2</summary>
    [ModbusRegister(Access.ReadWrite, Gpio = "Rev1 PD5, Rev2 PC6", Display = "bool"), ModbusMapping(Address = 21)]
    public ushort Valve2;

    /// <summary>Pneumatikventil 3</summary>
    [ModbusRegister(Access.ReadWrite, Gpio = "PD11", Display = "bool"), ModbusMapping(Address = 22)]
    public ushort Valve3;

    /// <summary>Vakuumventil (nur Hardware-Revision 2)</summary>
    [ModbusRegister(Access.ReadWrite, Gpio = "Rev2 PB14", Display = "bool"), ModbusMapping(Address = 23)]
    public ushort VacuumValve;

    /// <summary>Vakuumpumpe (nur Hardware-Revision 2)</summary>
    [ModbusRegister(Access.ReadWrite, Gpio = "Rev2 PC11", Display = "bool"), ModbusMapping(Address = 24)]
    public ushort VacuumPump;
}
