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

/// <summary>Raw LAN8742-PHY-Register (BSR/PHYSCSR/MCSR/SECR/SCSIR passiv, TCSR/CLR nach aktiver
/// TDR-Kabeldiagnose) -- unveraendert weitergereicht, jede Bit-Interpretation (Link-Status,
/// Speed/Duplex, Kabeltyp/-Laenge) passiert im Web-UI (web/src/apps/system-info-app.ts), s.
/// Core/Src/webserver.cpp's read_phy_registers()/read_tdr_registers().</summary>
[BinaryType]
public struct PhyRegisters
{
	public bool readOk;       // false: schon die passiven Reads schlugen fehl, alle Felder unten bedeutungslos
	public bool tdrAvailable; // false: tcsr/clr bedeutungslos (aktive TDR-Messung fehlgeschlagen)
	public ushort bsr;
	public ushort physcsr;
	public ushort mcsr;
	public ushort secr;       // Symbol Error Counter -- loescht sich beim Lesen selbst (Datenblatt)
	public ushort scsir;
	public ushort tcsr;
	public ushort clr;
}

/// <summary>Nachfolger von GET /api/system -- Laufzeitwerte, die sich seit dem letzten Aufruf
/// aendern koennen (Compile-Zeit-Konstanten wie Firmware-Version/Hostname/Chip-UID/MAC kommen
/// weiterhin einmalig ueber web/generated/build-info.ts, keine unnoetige Wire-Uebertragung).</summary>
[BinaryMessage(MessageKind.Request)]
public class SystemInfoRequest
{
}

[BinaryMessage(MessageKind.Response)]
public class SystemInfoMessage
{
	public uint uptimeSeconds;
	public uint freeHeapBytes;
	[BinaryCount(4)] public byte[] ipAddress = new byte[4]; // MSB-Oktett zuerst, wie IP_ADDR_FMT_ARGS
	[BinaryCount(4)] public byte[] netMask = new byte[4];
	public byte resetCauseCode; // s. App::ResetCauseCode()
	public PhyRegisters phy;
	[BinaryCount(16)] public byte[] i2c1Scan = new byte[16]; // Bitfeld, s. App::i2c1_scan
	[BinaryCount(16)] public byte[] i2c2Scan = new byte[16];
	[BinaryCount(16)] public byte[] i2c4Scan = new byte[16];
}
