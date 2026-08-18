using BestBinaryBuffers;

namespace system;

[BinaryType]
public enum LogLevel : byte
{
	LOG_TRACE = 0,
	LOG_DEBUG = 1,
	LOG_INFO = 2,
	LOG_WARN = 3,
	LOG_ERROR = 4,
	LOG_FATAL = 5,
}

[BinaryMessage(MessageKind.Event)]
public class LogMessage
{
	public LogLevel level;
	public uint timestampMs;
	public string text = "";
}
