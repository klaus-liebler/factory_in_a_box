using UniversalRegisterAccess;

[ModbusRegion("Motor-PWM")]
public class Motor
{
    /// <summary>Foerderband-Motor Duty</summary>
    [ModbusRegister(Access.ReadWrite, Unit = "Promille", Gpio = "TIM4_CH3/PD14", Display = "range", Min = 0, Max = 1000), ModbusMapping(Address = 40)]
    public ushort ConveyorPwm;

    /// <summary>Kompressor-Motor Duty</summary>
    [ModbusRegister(Access.ReadWrite, Unit = "Promille", Gpio = "TIM4_CH4/PD15", Display = "range", Min = 0, Max = 1000), ModbusMapping(Address = 41)]
    public ushort CompressorPwm;
}
