using UniversalRegisterAccess;

/// <summary>Malibotti-Status-Bitfeld (siehe MalibottiStatus.MalibottiStatus) -- Bit-Positionen, keine Registerwerte.</summary>
[ModbusType]
public enum MalibottiStatusBits : ushort
{
    /// <summary>Der Roboter meldet sich (ein Status innerhalb der letzten Sekunde).</summary>
    Connected = 0,
    /// <summary>Eine Mission laeuft.</summary>
    Busy = 1,
    /// <summary>Der Roboter meldet einen Fehler (Code in MalibottiMissionControl als -Wert).</summary>
    Error = 2,
}

[ModbusRegion("Malibotti Status")]
public class MalibottiStatus
{
    /// <summary>Status-Bitfeld (siehe MalibottiStatusBits)</summary>
    [ModbusRegister(Access.ReadOnly, Display = "binary"), ModbusMapping(Address = 260)]
    public MalibottiStatusBits MalibottiStatus;

    /// <summary>Die Mission, die der Roboter gerade abarbeitet (0: keine)</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 261)]
    public short MalibottiActiveMission;
}
