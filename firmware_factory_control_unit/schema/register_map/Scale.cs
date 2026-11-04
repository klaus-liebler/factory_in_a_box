using UniversalRegisterAccess;

[ModbusRegion("Waegezelle")]
public class Scale
{
    /// <summary>Gewicht Waegezelle A</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 220)]
    public int ScaleAWeight;

    /// <summary>0 = ok, ungleich 0 = Fehler (aktuell: 1 = keine gueltige Messung)</summary>
    [ModbusRegister(Access.ReadOnly, Display = "status"), ModbusMapping(Address = 222)]
    public ushort ScaleAStatus;

    /// <summary>Rohwert Waegezelle B (Reserve/Erweiterung)</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 223)]
    public uint ScaleBRaw;

    /// <summary>0 = ok, ungleich 0 = Fehler -- auf dieser Platinen-Revision nicht bestueckt,
    /// Register bleibt ungeschrieben (zeigt daher immer 0)</summary>
    [ModbusRegister(Access.ReadOnly, Display = "status"), ModbusMapping(Address = 225)]
    public ushort ScaleBStatus;
}
