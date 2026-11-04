using UniversalRegisterAccess;

[ModbusRegion("WS2812")]
public class Ws2812
{
    /// <summary>Index in fest im Code hinterlegte Farbmuster-Tabelle</summary>
    [ModbusRegister(Access.ReadWrite, Gpio = "PE5/TIM15_CH1"), ModbusMapping(Address = 100)]
    public ushort Ws2812Ch1Pattern;

    /// <summary>Index in fest im Code hinterlegte Farbmuster-Tabelle</summary>
    [ModbusRegister(Access.ReadWrite, Gpio = "PE6/TIM15_CH2"), ModbusMapping(Address = 101)]
    public ushort Ws2812Ch2Pattern;
}
