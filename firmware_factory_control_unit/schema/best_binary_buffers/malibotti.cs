using BestBinaryBuffers;

namespace malibotti;

// The UART link between the FCU and the Malibotti robot (ESP32-S3), FCU revision 3: USART6 on J16
// (FCU TX PC6 -> robot RX, FCU RX PC7 <- robot TX), 115200 Bd 8N1, 3.3 V levels. The robot keeps its
// missions itself (it is not taught); the FCU only says which one to run.
//
// Each message is the BestBinaryBuffers encoding as on the WebSocket - namespace id and type id
// (2 bytes each, little endian), then the fields - framed for the byte stream by rtos::serial
// (rtoos/include/rtos/serial/frame.hpp): COBS(message | CRC-16/CCITT-FALSE little endian) 0x00.
// A receiver throws away garbled frames and finds the next one at the next 0x00.
//
// The exchange (docs/malibotti-uart.md):
//   robot -> FCU  Status every 200 ms and at once on every change. No Status for 1 s: not connected.
//   FCU -> robot  Command with a new sequence number, repeated every 200 ms until a Status carries
//                 that number in ackSeq - so a lost frame in either direction costs one repetition,
//                 and a repeated command is never run twice (the robot acts once per seq).

[BinaryType]
public enum CommandKind : byte
{
	/// <summary>Only the acknowledgement: the robot answers with its Status (also after a restart of the FCU).</summary>
	Ping = 0,
	/// <summary>Run mission `mission` (refused when one runs: Status.lastResult = Busy).</summary>
	StartMission = 1,
	/// <summary>Stop the running mission (lastResult: Aborted).</summary>
	Abort = 2,
}

[BinaryType]
public enum RobotState : byte
{
	Idle = 0,
	Busy = 1,
	Error = 2,
}

/// <summary>The result of the last mission or command, as MalibottiMissionControl shows it negated.</summary>
[BinaryType]
public enum MissionResult : byte
{
	Ok = 0,
	NoAnswer = 1,       // set by the FCU, never sent: no Status
	UnknownMission = 2,
	Aborted = 3,
	Busy = 4,           // a start while a mission ran
	Blocked = 5,        // the robot could not go on (obstacle, timeout of a step)
	Fault = 6,          // a fault of the robot (drive, sensor, battery)
}

/// <summary>FCU -> robot. An event, not a request: the acknowledgement is ackSeq in the next Status.</summary>
[BinaryMessage(MessageKind.Event)]
public class Command
{
	public byte seq;              // 1..255, wraps; 0 is never used (the robot's ackSeq after its start)
	public CommandKind kind;
	public ushort mission;        // StartMission: 1..32767
}

/// <summary>Robot -> FCU, every 200 ms and on every change.</summary>
[BinaryMessage(MessageKind.Event)]
public class Status
{
	public byte ackSeq;           // the seq of the last command the robot took (0: none since its start)
	public RobotState state;
	public ushort activeMission;   // the mission it runs, 0: none
	public ushort lastMission;     // the mission lastResult belongs to
	public MissionResult lastResult;
	public ushort supplyMv;        // its battery or supply voltage, 0: unknown
}
