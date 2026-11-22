using BestBinaryBuffers;

namespace ethercat;

// The EtherCAT slave's status for the web UI's system page (src/ethercat.cpp status(), sent by
// src/web.cpp): the ESC's registers raw, as rtos::ethercat::Diagnostics reads them, plus the
// internal PHYs and the slave's counters. The page interprets the bits.

/// <summary>One EtherCAT port: the ESC's error counters and its internal PHY.</summary>
[BinaryType]
public struct EcatPort
{
	public byte invalidFrames;     // 0300h + 2n
	public byte rxErrors;          // 0301h + 2n
	public byte forwardedErrors;   // 0308h + n
	public byte lostLinks;         // 0310h + n
	public bool phyPresent;        // false: the PHY did not answer, the PHY fields are meaningless
	public byte phyAddress;
	public bool phyLink;
	public bool phyAutonegotiationComplete;
	public bool phyEnergy;         // a link partner sends (ENERGYON)
	public bool phyAutoMdix;
	public ushort phyBasicControl;
	public ushort phyBasicStatus;
}

[BinaryType]
public struct EcatSyncManager
{
	public ushort start;
	public ushort length;
	public byte control;
	public byte status;
	public byte activate;
	public byte pdiControl;
}

[BinaryType]
public struct EcatFmmu
{
	public uint logicalStart;
	public ushort length;
	public ushort physicalStart;
	public byte type;      // bit 0 read, bit 1 write (master's view)
	public bool active;
}

[BinaryMessage(MessageKind.Request)]
public class StatusRequest
{
}

[BinaryMessage(MessageKind.Response)]
public class StatusResponse
{
	public byte mode;              // 0 no ESC (or none on this revision), 1 LAN9252-compatible straps (not driven), 2 Direct Mapped
	public bool ready;             // HW_CFG READY: configuration loaded
	public bool irq;               // woken by the ESC's IRQ line (else polled)
	public uint idRev;             // LAN9253 ID_REV
	public uint hwCfg;
	public uint statusAgeMs;       // since the snapshot was taken
	public byte escType;
	public byte escRevision;
	public ushort escBuild;
	public byte fmmuCount;
	public byte syncManagerCount;
	public byte ramKb;
	public byte portDescriptor;
	public ushort features;
	public ushort stationAddress;
	public ushort stationAlias;
	public ushort dlStatus;
	public ushort alStatus;
	public ushort alStatusCode;
	public byte pdiControl;
	public byte escConfig;
	public byte processingUnitErrors;
	public byte pdiErrors;
	public ushort watchdogDivider;
	public ushort watchdogTimeProcessData;
	public byte watchdogCounterProcessData;
	public byte watchdogCounterPdi;
	public ushort eepromStatus;
	public ulong systemTimeNs;
	public uint eepromReads;
	public uint eepromWrites;
	public uint alTransitions;
	public uint alRefused;
	public ushort lastRefusedCode;
	public byte lastRefusedFrom;
	public byte lastRefusedTo;
	public uint mailboxRequests;
	public uint mailboxResponses;
	public uint mailboxErrors;      // mailbox error replies (unknown protocol, bad header)
	public uint mailboxRepeats;     // repeat requests of the master (a lost response)
	public uint sdoUploads;
	public uint sdoDownloads;
	public uint sdoAborts;
	public uint sdoInfoRequests;
	public uint lastAbortCode;
	public ushort lastAbortIndex;
	public byte lastAbortSubindex;
	public ushort outputsBytes;     // RxPDO bytes of the assignment (SM2)
	public ushort inputsBytes;      // TxPDO bytes (SM3)
	[BinaryMaxItemCount(2)] public EcatPort[] ports;
	[BinaryMaxItemCount(4)] public EcatSyncManager[] syncManagers;
	[BinaryMaxItemCount(3)] public EcatFmmu[] fmmus;
}
