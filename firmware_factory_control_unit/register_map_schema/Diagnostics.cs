using UniversalRegisterAccess;

/// <summary>Health-Bitfeld (siehe HealthState) -- Bit-Positionen, keine Registerwerte.</summary>
[ModbusType]
public enum HealthBits : ushort
{
    Ok = 0,
    Overtemperature = 1,
    PowerFault = 2,
    CanError = 3,
    EthLinkDown = 4,
    I2cSensorFault = 5,
    StepperFault = 6,
}

/// <summary>96-Bit Chip-UID (3x 32-Bit-Wort).</summary>
[ModbusType]
public struct Uid96
{
    public uint Word0;
    public uint Word1;
    public uint Word2;
}

[ModbusRegion("Diagnostik")]
public class Diagnostics
{
    /// <summary>Health-Bitfeld (siehe HealthBits)</summary>
    [ModbusRegister(Access.ReadOnly, Display = "binary"), ModbusMapping(Address = 0)]
    public HealthBits HealthState;

    /// <summary>96-Bit Chip-UID</summary>
    [ModbusRegister(Access.ReadOnly, Display = "hex"), ModbusMapping(Address = 1)]
    public Uid96 ChipId;

    /// <summary>Firmware-Version Major</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 7)]
    public ushort FwVersionMajor;

    /// <summary>Firmware-Version Minor</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 8)]
    public ushort FwVersionMinor;

    /// <summary>Firmware-Version Patch</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 9)]
    public ushort FwVersionPatch;

    /// <summary>Freilaufender Systick-Zaehler, Ueberlauf nach 65535</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "ticks"), ModbusMapping(Address = 10)]
    public ushort TimerTick;

    /// <summary>Freier newlib-Heap (High-Water-Mark bis zur MSP-Stack-Reserve, s. sysmem.c) --
    /// freigegebene, aber noch sbrk'te Bloecke zaehlen nicht mit, also eine pessimistische
    /// untere Schranke</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "KiB"), ModbusMapping(Address = 11)]
    public ushort FreeHeapKib;

    /// <summary>Chip-Temperatur (interner DTS-Sensor, RM0481 Kap. 29, werksseitig je Chip
    /// kalibriert)</summary>
    [ModbusRegister(Access.ReadOnly, Unit = "C"), ModbusMapping(Address = 12)]
    public short CpuTemperatureC;
}
