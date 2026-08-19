using BestBinaryBuffers;

namespace tasks;

// Werte entsprechen 1:1 TX_READY..TX_PRIORITY_CHANGE (libs/ST/threadx/common/inc/tx_api.h).
[BinaryType]
public enum ThreadState : byte
{
	Ready = 0,
	Completed = 1,
	Terminated = 2,
	Suspended = 3,
	Sleep = 4,
	QueueSuspended = 5,
	SemaphoreSuspended = 6,
	EventFlag = 7,
	BlockMemory = 8,
	ByteMemory = 9,
	IoDriver = 10,
	File = 11,
	TcpIp = 12,
	MutexSuspended = 13,
	PriorityChange = 14,
	Unknown = 255,
}

[BinaryType]
public struct TaskInfo
{
	[BinaryCount(32)] public byte[] name; // null-gepolstertes ASCII (tx_thread_name)
	public ThreadState state;
	public byte priority;
	public uint stackSizeBytes;
	public uint stackUsedCurrentBytes;
	public uint stackUsedHighWaterBytes;
	public uint runCount;
	public ushort cpuPermille; // 0..1000 = 0.0%..100.0%, gemittelt seit der letzten Anfrage
}

// Bewusst leer -- reiner Trigger, alle Daten kommen in TaskListMessage. Wird von der Web-UI nur
// gesendet, waehrend die Task-Manager-Seite geoeffnet ist (kein periodischer Firmware-Broadcast).
[BinaryMessage(MessageKind.Request)]
public class TaskManagerRequest
{
}

[BinaryMessage(MessageKind.Response)]
public class TaskListMessage
{
	// Bewusst NICHT "tasks" genannt (wie der umschliessende Namespace) -- der TS-Generator legt
	// beim Dekodieren eine gleichnamige lokale Variable an, die den Namespace-Bezeichner
	// "tasks" verdeckt und "tasks.decodeTaskInfo(...)" dadurch faelschlich als Zugriff auf die
	// (leere) Array-Property statt auf die Namespace-Funktion aufloest.
	public TaskInfo[] items;
}
