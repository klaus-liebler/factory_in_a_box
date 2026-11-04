using UniversalRegisterAccess;

/// <summary>RoArm-Status-Bitfeld (siehe RoArmStatus.Status) -- Bit-Positionen, keine Registerwerte.</summary>
[ModbusType]
public enum RoArmStatusBits : ushort
{
    TorqueEnabled = 0,
    Moving = 1,
    Error = 2,
    TeachModeActive = 3,
}

[ModbusRegion("RoArm-M3 Status")]
public class RoArmStatus
{
    /// <summary>Status-Bitfeld (siehe RoArmStatusBits)</summary>
    [ModbusRegister(Access.ReadOnly, Display = "binary"), ModbusMapping(Address = 250)]
    public RoArmStatusBits Status;

    /// <summary>Spiegel von RoArmMissionControl fuer lesende OPC-UA-/Modbus-Clients</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 251)]
    public short RoArmActiveMission;

    /// <summary>Aktueller Winkel Gelenk 1 (Base)</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "0.01deg"), ModbusMapping(Address = 252)]
    public short RoArmJoint1AngleCentiDeg;

    /// <summary>Aktueller Winkel Gelenk 2 (Shoulder)</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "0.01deg"), ModbusMapping(Address = 253)]
    public short RoArmJoint2AngleCentiDeg;

    /// <summary>Aktueller Winkel Gelenk 3 (Elbow)</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "0.01deg"), ModbusMapping(Address = 254)]
    public short RoArmJoint3AngleCentiDeg;

    /// <summary>Aktueller Winkel Gelenk 4 (Wrist)</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "0.01deg"), ModbusMapping(Address = 255)]
    public short RoArmJoint4AngleCentiDeg;

    /// <summary>Aktueller Winkel Gelenk 5 (Roll)</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "0.01deg"), ModbusMapping(Address = 256)]
    public short RoArmJoint5AngleCentiDeg;

    /// <summary>Aktueller Winkel Gelenk 6 (Gripper)</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "0.01deg"), ModbusMapping(Address = 257)]
    public short RoArmJoint6AngleCentiDeg;
}
