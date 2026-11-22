using UniversalRegisterAccess;

// Two ways to set the LEDs of a channel: a pattern (the pattern register) or single LEDs (the pixel
// register: index and RGB565 colour in one write). A pixel write starts from what the LEDs show and
// changes one; the pattern register then reads 0xFFFF until a pattern is written again (src/ws2812.hpp).
// The pixel register is a command: once done it reads 0xFFFFFFFF (ready), so every write is a change.
[ModbusRegion("WS2812")]
public class Ws2812
{
    /// <summary>Lichtmuster (0 aus, 1 rot, 2 gelb, 3 gruen, 4 blau/weiss); 0xFFFF: einzelne LEDs ueber WS2812_CH1_PIXEL gesetzt</summary>
    [ModbusRegister(Access.ReadWrite, Gpio = "PE5/TIM15_CH1"), ModbusMapping(Address = 100)]
    public ushort Ws2812Ch1Pattern;

    /// <summary>Farbe aus der Palette (0 schwarz, 1 rot, 2 gruen, 3 blau, 4 weiss, 5 gelb, 6 cyan, 7 magenta); 0xFFFF: ueber WS2812_CH2_PIXEL gesetzt</summary>
    [ModbusRegister(Access.ReadWrite, Gpio = "PE6/TIM15_CH2"), ModbusMapping(Address = 101)]
    public ushort Ws2812Ch2Pattern;

    /// <summary>Eine LED des Streifens setzen: hohes Wort LED-Index (0..7), niedriges Wort Farbe RGB565; beide Worte in einem Schreibvorgang; liest danach 0xFFFFFFFF (bereit)</summary>
    [ModbusRegister(Access.ReadWrite, Gpio = "PE5/TIM15_CH1", Display = "index-rgb565", Kind = RegisterKind.Command), ModbusMapping(Address = 102)]
    public uint Ws2812Ch1Pixel;

    /// <summary>Die einzelne LED setzen: hohes Wort LED-Index (0), niedriges Wort Farbe RGB565; beide Worte in einem Schreibvorgang; liest danach 0xFFFFFFFF (bereit)</summary>
    [ModbusRegister(Access.ReadWrite, Gpio = "PE6/TIM15_CH2", Display = "index-rgb565", Kind = RegisterKind.Command), ModbusMapping(Address = 104)]
    public uint Ws2812Ch2Pixel;
}
