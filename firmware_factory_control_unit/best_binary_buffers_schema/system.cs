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

/// <summary>One log record, sent to every connected browser the moment the firmware writes it
/// (the firmware keeps only a small buffer, the browser keeps the history: Terminal page).</summary>
[BinaryMessage(MessageKind.Event)]
public class LogMessage
{
	public LogLevel level;
	public uint sequence;    // consecutive; a gap = records lost on the way
	public uint uptimeMs;    // since boot
	public ulong unixMs;     // wall clock (SNTP), 0 while not synchronised
	public ushort lost;      // records the firmware's buffer dropped right before this one
	[BinaryMaxEncodedByteLength(16)] public string logger = "";  // source module (file name)
	[BinaryMaxEncodedByteLength(119)] public string text = "";   // the firmware's record holds 119 characters
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
