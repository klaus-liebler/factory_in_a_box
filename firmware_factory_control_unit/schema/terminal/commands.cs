using TerminalSchema;

namespace terminal;

// Commands of the web UI's Terminal page (docs/terminal.md). The builder (GenerateTerminalFiles: TerminalCatalog in FirmwareBuilder.Common)
// generates web/generated/terminal-commands.ts (completion, help, tooltip: compiled into the web app) and
// generated/terminal_commands.hh (the firmware needs a handler for every command that is not Client and
// an answer for every lookup, checked at compile time).
//
// A command is a class, its parameters are its fields in order; the /// summary is the help text.
// string? marks an optional parameter. A parameter's values are either listed ([Value]), fetched live
// from the device ([Lookup]: answered by src/terminal.cpp), computed in the browser ([Source]), or free.

/// <summary>Values the browser fetches live from the device (terminal.LookupRequest.lookup).</summary>
[TerminalLookups]
public enum Lookup
{
	/// <summary>digital inputs of this hardware revision with their levels</summary>
	inputs,
	/// <summary>switchable outputs of this hardware revision with their states</summary>
	outputs,
	/// <summary>all GPIO pins of the package with their levels</summary>
	pins,
}

/// <summary>list the commands, or show the parameters of one command</summary>
[TerminalCommand(Client = true)]
public class help
{
	/// <summary>a command name</summary>
	[Source(Source.commands)] public string? command;
}

/// <summary>clear the console (also Ctrl+L)</summary>
[TerminalCommand(Client = true)]
public class clear;

/// <summary>host name, hardware revision, firmware, uptime, Ethernet</summary>
[TerminalCommand]
public class info;

/// <summary>digital inputs with their levels</summary>
[TerminalCommand]
public class @in
{
	/// <summary>one input; without: all</summary>
	[Lookup(Lookup.inputs)] public string? input;
}

/// <summary>outputs (valves, vacuum, stepper enable): list or switch</summary>
[TerminalCommand]
public class @out
{
	/// <summary>output to switch; without: list all</summary>
	[Lookup(Lookup.outputs)] public string? output;

	/// <summary>new state</summary>
	[Value("on"), Value("off")] public string? state;
}

/// <summary>motor duty cycle</summary>
[TerminalCommand]
public class pwm
{
	/// <summary>PWM output</summary>
	[Value("conveyor", "conveyor belt motor (TIM4_CH3)"), Value("compressor", "compressor motor")] public string motor;

	/// <summary>0..1000 (per mille)</summary>
	public string duty;
}

/// <summary>read any pin, or drive an output pin</summary>
[TerminalCommand]
public class gpio
{
	/// <summary>pin name, e.g. PC7</summary>
	[Lookup(Lookup.pins)] public string pin;

	/// <summary>drive the pin (an output of the I/O cycle is overwritten within 50 ms)</summary>
	[Value("1", "high"), Value("0", "low")] public string? level;
}

/// <summary>pressure sensor raw value (ADC1)</summary>
[TerminalCommand]
public class adc;

/// <summary>supply: INA226 voltage, current, power</summary>
[TerminalCommand]
public class power;

/// <summary>CPU temperature</summary>
[TerminalCommand]
public class temp;

/// <summary>ToF distances, colour sensor, scale</summary>
[TerminalCommand]
public class sensors;

/// <summary>scan an I2C bus for devices</summary>
[TerminalCommand]
public class i2c
{
	/// <summary>I2C bus</summary>
	[Value("1", "I2C1"), Value("2", "I2C2 (revisions 1/2)"), Value("3", "I2C3 (revision 3)"), Value("4", "I2C4")] public string bus;
}

/// <summary>read a register, or write a holding register</summary>
[TerminalCommand]
public class reg
{
	/// <summary>register bank</summary>
	[Value("h", "holding register (read/write)"), Value("i", "input register (read only)")] public string bank;

	/// <summary>register address (type a name to search)</summary>
	[Source(Source.registers)] public string address;

	/// <summary>new value -32768..65535 (holding registers only)</summary>
	public string? value;
}

/// <summary>enable the steppers and move one to a position</summary>
[TerminalCommand]
public class stepper
{
	/// <summary>stepper</summary>
	[Value("1"), Value("2")] public string motor;

	/// <summary>target position in steps (signed)</summary>
	public string target;
}

/// <summary>home a stepper: sensorless (StallGuard) towards its end stop, which becomes position 0</summary>
[TerminalCommand]
public class home
{
	/// <summary>stepper</summary>
	[Value("1"), Value("2")] public string motor;
}

/// <summary>WS2812 LED pattern</summary>
[TerminalCommand]
public class led
{
	/// <summary>LED channel</summary>
	[Value("1", "channel 1: pattern 0..4"), Value("2", "channel 2: colour 0..7")] public string channel;

	/// <summary>pattern number</summary>
	public string pattern;
}

/// <summary>IEC 61131-3 PLC: without arguments its state; run, stop, reset, control source, autostart (the PLC
/// thread runs the command and logs the result)</summary>
[TerminalCommand]
public class plc
{
	/// <summary>what to do</summary>
	[Value("run", "start the program (STOP -> RUN)"), Value("stop", "stop it, outputs to the safe state"),
	 Value("reset", "reload the program from the flash slot: warm | cold"),
	 Value("source", "who controls the outputs: fieldbus | iec (stored)"),
	 Value("autostart", "RUN after a restart: on | off (stored)"),
	 Value("unforce", "release every forced value (also a browser's debug lease)"),
	 Value("continue", "a halted program (breakpoint, step) runs on")] public string? action;
	/// <summary>reset: warm | cold; source: fieldbus | iec; autostart: on | off</summary>
	[Value("warm"), Value("cold"), Value("fieldbus"), Value("iec"), Value("on"), Value("off")] public string? option;
}

/// <summary>restart the controller</summary>
[TerminalCommand]
public class reset
{
	/// <summary>confirmation</summary>
	[Value("now")] public string confirm;
}
