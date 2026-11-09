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
