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
	// Matches TASK_COUNT in task_monitor.cpp -- the fixed list of App-owned ThreadX threads this
	// snapshot ever covers.
	[BinaryMaxItemCount(6)] public TaskInfo[] items;
}

/// <summary>Ein ThreadX-Byte-Pool oder NetX-Packet-Pool -- capacity/free sind je nach Pool-Art in
/// unterschiedlichen Einheiten zu verstehen (Byte-Pool: Bytes; Packet-Pool: Paketanzahl), daher
/// traegt "name" die Einheit direkt mit (z.B. "Byte-Pool (Bytes)"/"OPC UA Pakete (Pakete)") statt
/// eines eigenen Einheiten-Feldes -- schliesst das Instrumentierungs-Loch aus der RAM-Analyse
/// (bislang war keine dieser Pool-Groessen zur Laufzeit sichtbar, nur aus dem Quellcode geschaetzt).</summary>
[BinaryType]
public struct PoolInfo
{
	[BinaryCount(24)] public byte[] name;
	public uint capacity;
	public uint free;
}

[BinaryMessage(MessageKind.Request)]
public class PoolListRequest
{
}

[BinaryMessage(MessageKind.Response)]
public class PoolListMessage
{
	// App::byte_pool, App::packet_pool, Http::WebServer::PacketPool(), App::opcua_packet_pool --
	// s. HandlePoolListRequest() in task_monitor.cpp.
	[BinaryMaxItemCount(4)] public PoolInfo[] pools = System.Array.Empty<PoolInfo>();
	public uint freeHeapBytes; // newlib-Heap, s. GetFreeHeapBytes() (sysmem.c)
}
