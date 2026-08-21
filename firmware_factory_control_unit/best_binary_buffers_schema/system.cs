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
	// Matches the stack-local "char buf[256]" in log_write_line() (stm32_libs/common_stm32/log.c)
	// that log_log() formats the WebSocket-bound (unprefixed) message text into before truncating
	// -- 255 usable bytes (buf's last slot is reserved for vsnprintf's own NUL), so a longer line
	// can never actually reach this sink regardless of what the caller's format string produces.
	[BinaryMaxEncodedByteLength(255)] public string text = "";
}
