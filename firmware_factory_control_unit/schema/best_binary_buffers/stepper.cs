using BestBinaryBuffers;

namespace stepper;

// The stepper test bench of the web UI (page "Schrittmotoren"): both TMC2209 set up, moved, homed and
// watched. Answered by the I/O thread, which owns the TMC2209 UART and the step timers. Steps are
// micro steps (the STEP edges: TMC2209 DEDGE), rates micro steps per second.

[BinaryType]
public enum RampShape : byte
{
	Linear = 0,
	Sigmoid = 1,
}

/// <summary>The TMC2209 settings of one axis (datasheet 5). The firmware keeps them (most of these
/// registers cannot be read back) and writes all of them on SettingsRequest.</summary>
[BinaryType]
public struct TmcSettings
{
	public byte irun;            // 0..31: run current, (irun+1)/32 of the full scale
	public byte ihold;           // 0..31: standstill current
	public byte iholdDelay;      // 0..15: the ramp down to ihold after standstill, 2^18 clocks per step
	public byte tpowerdown;      // 0..255: standstill delay before the ramp down, 2^18 clocks (21.8 ms) each
	public bool spreadCycle;     // false: StealthChop (quiet), true: SpreadCycle
	public uint tpwmThrs;        // StealthChop below this TSTEP-velocity only (0: always)
	public bool pwmAutoscale;    // StealthChop: automatic current regulation
	public bool pwmAutograd;     // StealthChop: automatic gradient adaptation
	public ushort microsteps;    // 1, 2, 4 .. 256
	public byte sgThrs;          // StallGuard threshold (homing: stall when SG_RESULT <= 2 x sgThrs)
	public uint tcoolThrs;       // CoolStep and the stall output above this velocity (TSTEP <= tcoolThrs)
	public bool coolStep;        // CoolStep on (semin 0: off)
	public byte semin;           // 1..15: below semin*32 the current goes up
	public byte semax;           // 0..15: above (semin+semax+1)*32 it goes down
	public byte seup;            // 0..3: current up step 1, 2, 4, 8
	public byte sedn;            // 0..3: down speed 32, 8, 2, 1 StallGuard values per step
	public bool seimin;          // minimum current 1/4 (false: 1/2) of irun
}

/// <summary>The speed profile of the moves of one axis.</summary>
[BinaryType]
public struct Profile
{
	public uint startHz;         // micro steps/s at the start and the end
	public uint cruiseHz;        // micro steps/s after the ramp
	public uint accel;           // micro steps/s^2 of the linear ramp (the sigmoid: the same distance)
	public RampShape shape;
}

/// <summary>Read (write false) or write and read the settings and the profile of an axis.</summary>
[BinaryMessage(MessageKind.Request)]
public class SettingsRequest
{
	public byte axis;            // 0, 1
	public bool write;
	public TmcSettings settings;
	public Profile profile;
}

[BinaryMessage(MessageKind.Response)]
public class SettingsResponse
{
	public bool ok;
	[BinaryMaxEncodedByteLength(120)] public string message = "";
	public TmcSettings settings;
	public Profile profile;
}

/// <summary>Manual control of an axis: the register targets (Modbus, OPC UA, PLC) rest, this page
/// moves it - also without homing. Off: the target becomes the position (no jump).</summary>
[BinaryMessage(MessageKind.Request)]
public class ManualRequest
{
	public byte axis;
	public bool on;
}

[BinaryMessage(MessageKind.Response)]
public class ManualResponse
{
	public bool ok;
	[BinaryMaxEncodedByteLength(120)] public string message = "";
}

/// <summary>A relative move with the axis' profile (manual control only).</summary>
[BinaryMessage(MessageKind.Request)]
public class MoveRequest
{
	public byte axis;
	public int steps;
}

[BinaryMessage(MessageKind.Response)]
public class MoveResponse
{
	public bool ok;
	[BinaryMaxEncodedByteLength(120)] public string message = "";
}

/// <summary>Stop a move or a homing at once.</summary>
[BinaryMessage(MessageKind.Request)]
public class StopRequest
{
	public byte axis;
}

[BinaryMessage(MessageKind.Response)]
public class StopResponse
{
	public bool ok;
}

/// <summary>Sensorless homing with these parameters (the axis' SGTHRS setting is not changed).</summary>
[BinaryMessage(MessageKind.Request)]
public class HomeRequest
{
	public byte axis;
	public short speed;          // full steps/s towards the end stop (sign: direction)
	public byte sgThrs;
	public ushort timeoutMs;
}

[BinaryMessage(MessageKind.Response)]
public class HomeResponse
{
	public bool ok;
	[BinaryMaxEncodedByteLength(120)] public string message = "";
}

/// <summary>What the axis and its TMC2209 do now (the page asks about 10 times a second).</summary>
[BinaryMessage(MessageKind.Request)]
public class StatusRequest
{
	public byte axis;
}

[BinaryMessage(MessageKind.Response)]
public class StatusResponse
{
	public bool tmcOk;           // the TMC2209 answered all reads
	public int position;         // micro steps
	public bool moving;
	public bool manual;
	public bool homing;
	public bool homed;
	public byte homingResult;    // 0 none, 1 done, 2 timeout, 3 error
	public bool stallByDiag;     // the last homing stopped by the DIAG signal (else: SG_RESULT over UART)
	public bool hasDiag;         // the board has the DIAG wires (revisions 2, 3)
	public bool enabled;         // the drivers are enabled
	public uint drvStatus;       // DRV_STATUS (cs_actual, stst, stealth, ot, s2g, ola ...)
	public ushort sgResult;      // SG_RESULT
	public uint tstep;           // TSTEP (time per 1/256 step in clocks; 0xFFFFF: standstill)
	public uint pwmScale;        // PWM_SCALE (pwm_scale_sum, pwm_scale_auto)
	public byte gstat;           // GSTAT (reset, drv_err, uv_cp)
	public uint uptimeMs;        // when the values were read
}
