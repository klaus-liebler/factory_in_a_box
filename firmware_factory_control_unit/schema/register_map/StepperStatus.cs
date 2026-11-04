using UniversalRegisterAccess;

[ModbusRegion("Stepper-Status")]
public class StepperStatus
{
    /// <summary>Aktuelle Position Stepper 1</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 240)]
    public int Stepper1Position;

    /// <summary>Bit0 Enabled, Bit1 Moving, Bit2 Homing fehlgeschlagen, Bit3 InPosition, Bit4 Homing laeuft,
    /// Bit5 referenziert (Homing erfolgreich, beim Start oder per StartHoming)</summary>
    [ModbusRegister(Access.ReadOnly, Display = "binary"), ModbusMapping(Address = 242)]
    public ushort Stepper1Status;

    /// <summary>Aktuelle Position Stepper 2</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 243)]
    public int Stepper2Position;

    /// <summary>Bit0 Enabled, Bit1 Moving, Bit2 Homing fehlgeschlagen, Bit3 InPosition, Bit4 Homing laeuft,
    /// Bit5 referenziert (Homing erfolgreich, beim Start oder per StartHoming)</summary>
    [ModbusRegister(Access.ReadOnly, Display = "binary"), ModbusMapping(Address = 245)]
    public ushort Stepper2Status;
}
