using BestBinaryBuffers;

namespace plc;

// The IEC 61131-3 PLC (docs/plc-iec61131.md): status, control, program download and reading the
// program's variables (online values). The download comes in chunks (UploadRequest) into the
// device's RAM, then CommitRequest checks the image and writes it into the flash slots. Requests
// that change the PLC are answered by the PLC thread (through the outbox), the others by the
// network thread.

[BinaryType]
public enum PlcState : byte
{
	NoProgram = 0,
	Stop = 1,
	Run = 2,
	Error = 3,
	/// <summary>A breakpoint or a step stopped the program inside a cycle; the I/O cycle runs on, the
	/// outputs keep their values; it continues only on Continue or a step (never by itself).</summary>
	Halted = 4,
}

[BinaryType]
public enum ControlSource : byte
{
	/// <summary>Modbus, OPC UA and the web UI switch the outputs (as without a PLC).</summary>
	Fieldbus = 0,
	/// <summary>The IEC program owns the outputs; external writes to them are refused.</summary>
	Iec = 1,
}

[BinaryType]
public enum PlcCommand : byte
{
	Run = 1,
	Stop = 2,
	ResetWarm = 3,
	ResetCold = 4,
	SourceFieldbus = 5,
	SourceIec = 6,
	AutostartOn = 7,
	AutostartOff = 8,
	/// <summary>Release every forced value (and the debug lease); the terminal's `plc unforce`.</summary>
	UnforceAll = 9,
	/// <summary>HALTED: run on (only breakpoints stop); the terminal's `plc continue`.</summary>
	Continue = 10,
	/// <summary>HALTED: to the next statement - into calls, over them, out of the current POU.</summary>
	StepInto = 11,
	StepOver = 12,
	StepOut = 13,
}

[BinaryMessage(MessageKind.Request)]
public class StatusRequest
{
}

[BinaryMessage(MessageKind.Response)]
public class StatusResponse
{
	public PlcState state;
	public ControlSource source;
	public bool autostart;
	/// <summary>The cycle in ms (the I/O cycle).</summary>
	public ushort cycleMs;
	public uint cycles;
	/// <summary>VM instructions of the last cycle and the maximum since RUN.</summary>
	public uint lastInstructions;
	public uint maxInstructions;
	/// <summary>Bytes of the loaded image (0: none), its project hash (links it to the IDE's project).</summary>
	public uint imageBytes;
	public uint projectHash;
	public uint ioMapHash;
	/// <summary>ERROR: trap code (rtos::plc::Trap), TRAP operand and the program counter.</summary>
	public byte trap;
	public ushort trapCode;
	public uint trapPc;
	/// <summary>The largest image the flash slots hold and the data segment of the VM.</summary>
	public uint maxImageBytes;
	public uint dataBytes;
	/// <summary>Why downloads are refused right now ("" if they are not).</summary>
	[BinaryMaxEncodedByteLength(120)] public string storeProblem = "";
	/// <summary>Forced values on the device (any browser's); 0 after STOP, a switch, a download.</summary>
	public byte forces;
	/// <summary>Another browser holds the debug lease (forcing): this one may watch, not change.</summary>
	public bool leasedByOther;
}

[BinaryMessage(MessageKind.Request)]
public class ControlRequest
{
	public PlcCommand command;
}

[BinaryMessage(MessageKind.Response)]
public class ControlResponse
{
	public bool ok;
	[BinaryMaxEncodedByteLength(120)] public string message = "";
}

/// <summary>A chunk of the image at `offset`; offset 0 starts a new download (the PLC must be in STOP
/// or without a program). Chunks come in order.</summary>
[BinaryMessage(MessageKind.Request)]
public class UploadRequest
{
	public uint totalBytes;
	public uint offset;
	[BinaryMaxItemCount(1024)] public byte[] data = System.Array.Empty<byte>();
}

[BinaryMessage(MessageKind.Response)]
public class UploadResponse
{
	public bool ok;
	[BinaryMaxEncodedByteLength(120)] public string message = "";
}

/// <summary>All chunks are there: check the image (VM verifier), write it into the flash slot, load
/// it (STOP). Answered when the flash is written (seconds on the STM32H743).</summary>
[BinaryMessage(MessageKind.Request)]
public class CommitRequest
{
	public uint totalBytes;
	/// <summary>CRC-32 of the whole image, as the image's own header.</summary>
	public uint crc;
}

[BinaryMessage(MessageKind.Response)]
public class CommitResponse
{
	public bool ok;
	[BinaryMaxEncodedByteLength(120)] public string message = "";
}

/// <summary>Bytes of the program's data segment (online values, watch lists).</summary>
[BinaryMessage(MessageKind.Request)]
public class ReadRequest
{
	public uint address;
	public ushort length;
}

[BinaryMessage(MessageKind.Response)]
public class ReadResponse
{
	public bool ok;
	public uint address;
	[BinaryMaxItemCount(1024)] public byte[] data = System.Array.Empty<byte>();
}

/// <summary>Writes bytes into the program's data segment once, between two cycles (prepared values).</summary>
[BinaryMessage(MessageKind.Request)]
public class WriteRequest
{
	public uint address;
	[BinaryMaxItemCount(64)] public byte[] data = System.Array.Empty<byte>();
}

[BinaryMessage(MessageKind.Response)]
public class WriteResponse
{
	public bool ok;
}

/// <summary>A chunk of the project source (the IDE's project JSON) for the SD card,
/// /PLC/project.fcuplc.json (docs/plc-iec61131.md 4.5). Offset 0 starts, chunks come in order; the last
/// one replaces the stored project - written as /PLC/project.tmp and renamed once complete, so a power
/// loss keeps the old project or the new one. Answered by the I/O thread, which owns the card.</summary>
[BinaryMessage(MessageKind.Request)]
public class PutProjectRequest
{
	public uint totalBytes;
	public uint offset;
	[BinaryMaxItemCount(1024)] public byte[] data = System.Array.Empty<byte>();
}

[BinaryMessage(MessageKind.Response)]
public class PutProjectResponse
{
	public bool ok;
	[BinaryMaxEncodedByteLength(120)] public string message = "";
}

/// <summary>A chunk of the stored project source from `offset` ("Upload from device").</summary>
[BinaryMessage(MessageKind.Request)]
public class GetProjectRequest
{
	public uint offset;
}

[BinaryMessage(MessageKind.Response)]
public class GetProjectResponse
{
	/// <summary>false: no card, no stored project, or a read error (message).</summary>
	public bool ok;
	public uint totalBytes;
	public uint offset;
	[BinaryMaxItemCount(1024)] public byte[] data = System.Array.Empty<byte>();
	[BinaryMaxEncodedByteLength(120)] public string message = "";
}

/// <summary>Force a variable of the program's data segment (docs/plc-iec61131.md 10.3): its bytes are
/// written after the inputs are read and again after the program ran, every cycle, until released.
/// Takes the debug lease: while one browser forces, the others may watch, not change. Every force is
/// released on STOP, a switch of the control source, a reset and a download.</summary>
[BinaryMessage(MessageKind.Request)]
public class ForceRequest
{
	public uint address;
	[BinaryMaxItemCount(8)] public byte[] data = System.Array.Empty<byte>();
}

[BinaryMessage(MessageKind.Response)]
public class ForceResponse
{
	public bool ok;
	[BinaryMaxEncodedByteLength(120)] public string message = "";
}

/// <summary>Release the force at `address`, or every force with 0xFFFFFFFF (which also ends the lease).</summary>
[BinaryMessage(MessageKind.Request)]
public class UnforceRequest
{
	public uint address;
}

[BinaryMessage(MessageKind.Response)]
public class UnforceResponse
{
	public bool ok;
	[BinaryMaxEncodedByteLength(120)] public string message = "";
}

/// <summary>The breakpoints: code offsets of statement starts (the IDE maps lines to them with its debug
/// information), replacing the earlier ones. Non-empty takes the debug lease. STOP and a download
/// clear them.</summary>
[BinaryMessage(MessageKind.Request)]
public class BreakpointsRequest
{
	[BinaryMaxItemCount(64)] public uint[] offsets = System.Array.Empty<uint>();
}

[BinaryMessage(MessageKind.Response)]
public class BreakpointsResponse
{
	public bool ok;
	[BinaryMaxEncodedByteLength(120)] public string message = "";
}

/// <summary>Where the program halted: the program counter and the call chain.</summary>
[BinaryMessage(MessageKind.Request)]
public class HaltRequest
{
}

[BinaryMessage(MessageKind.Response)]
public class HaltResponse
{
	/// <summary>false: not halted (the other fields are empty).</summary>
	public bool halted;
	/// <summary>1 breakpoint, 2 step.</summary>
	public byte reason;
	public uint pc;
	/// <summary>The return offsets of the active calls, the program's own call first (its value is
	/// 0xFFFFFFFF): the IDE turns them into the call stack.</summary>
	[BinaryMaxItemCount(32)] public uint[] returnPcs = System.Array.Empty<uint>();
}

/// <summary>Watch subscription (docs/plc-iec61131.md 10.3): ranges of the program's data segment the
/// browser shows, replacing its earlier list; empty ends it. The PLC thread then sends WatchValues to
/// this session after a cycle, at most every 100 ms (in STOP/HALTED/ERROR as well, at the same rate).
/// `generation` is echoed, so the browser knows which list the values belong to. At most 128 ranges and
/// 1024 bytes in all; the session's subscription ends when its WebSocket closes.</summary>
[BinaryMessage(MessageKind.Request)]
public class WatchRequest
{
	public ushort generation;
	[BinaryMaxItemCount(128)] public uint[] addresses = System.Array.Empty<uint>();
	[BinaryMaxItemCount(128)] public ushort[] lengths = System.Array.Empty<ushort>();
}

[BinaryMessage(MessageKind.Response)]
public class WatchResponse
{
	public bool ok;
	[BinaryMaxEncodedByteLength(120)] public string message = "";
}

/// <summary>The values of a subscription: the bytes of its ranges one after the other, read by the PLC
/// thread between two cycles (consistent: all from the end of the same cycle).</summary>
[BinaryMessage(MessageKind.Event)]
public class WatchValues
{
	public ushort generation;
	public uint cycles;
	public PlcState state;
	[BinaryMaxItemCount(1024)] public byte[] data = System.Array.Empty<byte>();
}
