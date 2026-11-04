using BestBinaryBuffers;

namespace terminal;

/// <summary>Live values of a command parameter (schema/terminal/commands.cs, enum Lookup):
/// one line per value, value and description separated by a tab.</summary>
[BinaryMessage(MessageKind.Request)]
public class LookupRequest
{
	public byte lookup;  // index into "lookups" (terminal_catalog::Lookup)
}

[BinaryMessage(MessageKind.Response)]
public class LookupResponse
{
	public bool ok;  // false: unknown lookup
	[BinaryMaxEncodedByteLength(1100)] public string values = "";
}

/// <summary>One command line of the browser terminal, executed by the firmware's I/O thread.</summary>
[BinaryMessage(MessageKind.Request)]
public class CommandRequest
{
	[BinaryMaxEncodedByteLength(200)] public string line = "";
}

[BinaryMessage(MessageKind.Response)]
public class CommandResponse
{
	public bool ok;  // false: unknown command or wrong arguments
	[BinaryMaxEncodedByteLength(1100)] public string output = "";
}
