using UniversalRegisterAccess;

[ModbusRegion("Stepper-Steuerung")]
public class StepperCtrl
{
    /// <summary>Stepper-Treiber global freigeben</summary>
    [ModbusRegister(Access.ReadWrite, Gpio = "PB7", Display = "bool"), ModbusMapping(Address = 60)]
    public ushort StepperEnable;

    /// <summary>Zielposition Stepper 1</summary>
    [ModbusRegister(Access.ReadWrite), ModbusMapping(Address = 61)]
    public int Stepper1Target;

    /// <summary>Geschwindigkeit Stepper 1</summary>
    [ModbusRegister(Access.ReadWrite, Unit = "Schritte/s"), ModbusMapping(Address = 63)]
    public ushort Stepper1Speed;

    /// <summary>Beschleunigung Stepper 1</summary>
    [ModbusRegister(Access.ReadWrite, Unit = "Schritte/s^2"), ModbusMapping(Address = 64)]
    public ushort Stepper1Accel;

    /// <summary>Zielposition Stepper 2</summary>
    [ModbusRegister(Access.ReadWrite), ModbusMapping(Address = 65)]
    public int Stepper2Target;

    /// <summary>Geschwindigkeit Stepper 2</summary>
    [ModbusRegister(Access.ReadWrite, Unit = "Schritte/s"), ModbusMapping(Address = 67)]
    public ushort Stepper2Speed;

    /// <summary>Beschleunigung Stepper 2</summary>
    [ModbusRegister(Access.ReadWrite, Unit = "Schritte/s^2"), ModbusMapping(Address = 68)]
    public ushort Stepper2Accel;

    /// <summary>1=Homing starten (stoppt Stepper1, sensorloses Homing per StallGuard), 0=erfolgreich
    /// beendet, &gt;1=Fehlercode</summary>
    [ModbusRegister(Access.ReadWrite, Min = 0, Max = 255), ModbusMapping(Address = 69)]
    public ushort Stepper1StartHoming;

    /// <summary>1=Homing starten (stoppt Stepper2, sensorloses Homing per StallGuard), 0=erfolgreich
    /// beendet, &gt;1=Fehlercode</summary>
    [ModbusRegister(Access.ReadWrite, Min = 0, Max = 255), ModbusMapping(Address = 70)]
    public ushort Stepper2StartHoming;
}
