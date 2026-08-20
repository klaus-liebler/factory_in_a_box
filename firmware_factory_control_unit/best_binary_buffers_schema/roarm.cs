using BestBinaryBuffers;

namespace roarm;

[BinaryType]
public enum ServoStatusBits : byte
{
    Ok = 0,
    CommError = 1,
    Overload = 2,
    Overheat = 3,
    Undervoltage = 4,
}

/// <summary>Aktiviert den Teach-Modus (sperrt Mission-Playback, armiert die Jog-Tracker und die
/// periodische PoseFeedback-Aussendung).</summary>
[BinaryMessage(MessageKind.Request)]
public class StartTeachModeRequest
{
}

[BinaryMessage(MessageKind.Response)]
public class StartTeachModeResponse
{
    public bool success;
}

[BinaryMessage(MessageKind.Request)]
public class StopTeachModeRequest
{
}

[BinaryMessage(MessageKind.Response)]
public class StopTeachModeResponse
{
    public bool success;
}

/// <summary>Setzt neue Ziel-Gelenkwinkel waehrend des Teach-Modus -- beliebig oft/schnell sendbar
/// (z.B. bei jeder Slider-Bewegung); die Firmware naehert sich per JointTracker glatt an (siehe
/// roarm_motion.hh). Event statt Request/Response, da kein Umlauf pro Nachricht noetig ist.</summary>
[BinaryMessage(MessageKind.Event)]
public class JointJogTarget
{
    [BinaryCount(6)]
    public short[] jointAnglesCentiDeg = new short[6];
}

/// <summary>Wie JointJogTarget, aber als kartesische Pose -- die Firmware macht IK
/// (roarm_kinematics.hh) und setzt daraus die Gelenk-Ziele.</summary>
[BinaryMessage(MessageKind.Event)]
public class CartesianJogTarget
{
    public short xMm;
    public short yMm;
    public short zMm;
    public short pitchCentiDeg;
    public short rollCentiDeg;
    public short gripperCentiDeg;
}

/// <summary>Laufende Ist-Werte waehrend des Teach-Modus, ca. alle 100ms gesendet.</summary>
[BinaryMessage(MessageKind.Event)]
public class PoseFeedback
{
    [BinaryCount(6)]
    public short[] jointAnglesCentiDeg = new short[6];
    public short xMm;
    public short yMm;
    public short zMm;
    public short pitchCentiDeg;
    public short rollCentiDeg;
    [BinaryCount(7)]
    public ServoStatusBits[] servoStatus = new ServoStatusBits[7];
}

[BinaryMessage(MessageKind.Request)]
public class ListMissionsRequest
{
}

/// <summary>Nur eine Variante (MissionSummary) implementiert dieses Interface -- BinaryUnion wird
/// hier ausschliesslich gebraucht, weil ein Array aus direkten Klassenreferenzen vom Generator
/// nicht unterstuetzt wird (siehe BestBinaryBuffers-README).</summary>
[BinaryUnion]
public interface IMissionSummary
{
}

[BinaryType]
public class MissionSummary : IMissionSummary
{
    public ushort missionIndex;
    public string name = "";
}

[BinaryMessage(MessageKind.Response)]
public class ListMissionsResponse
{
    public IMissionSummary[] missions = System.Array.Empty<IMissionSummary>();
}

[BinaryMessage(MessageKind.Request)]
public class GetMissionRequest
{
    public ushort missionIndex;
}

[BinaryMessage(MessageKind.Response)]
public class GetMissionResponse
{
    public bool found;
    public ushort missionIndex;
    public string name = "";
    public IMissionStep[] steps = System.Array.Empty<IMissionStep>();
}

[BinaryMessage(MessageKind.Request)]
public class SaveMissionRequest
{
    public ushort missionIndex;
    public string name = "";
    public IMissionStep[] steps = System.Array.Empty<IMissionStep>();
}

[BinaryMessage(MessageKind.Response)]
public class SaveMissionResponse
{
    public bool success;
    public short errorCode;
}

[BinaryMessage(MessageKind.Request)]
public class DeleteMissionRequest
{
    public ushort missionIndex;
}

[BinaryMessage(MessageKind.Response)]
public class DeleteMissionResponse
{
    public bool success;
}

/// <summary>Liefert die Namen der Eintraege aus der Mission-GPIO-Tabelle (siehe
/// roarm_mission_gpio.hh) fuer das GPIO-Schritt-Dropdown im Teach-UI.</summary>
[BinaryMessage(MessageKind.Request)]
public class GetMissionGpioListRequest
{
}

[BinaryMessage(MessageKind.Response)]
public class GetMissionGpioListResponse
{
    public string[] names = System.Array.Empty<string>();
}
