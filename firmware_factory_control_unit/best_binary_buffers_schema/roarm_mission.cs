using BestBinaryBuffers;

namespace roarm;

/// <summary>Ein einzelner Schritt einer Mission -- konkrete Varianten ueber BinaryUnion.</summary>
[BinaryUnion]
public interface IMissionStep
{
}

/// <summary>Bewegt alle 6 Gelenke auf die angegebenen Zielwinkel -- die eigentliche Fahrt wird
/// durch das trapezfoermige Geschwindigkeitsprofil in roarm_motion.hh geglaettet; dieser Schritt
/// gilt als abgeschlossen, sobald alle 6 JointTracker ihr Ziel erreicht haben.</summary>
[BinaryType]
public class JointMoveStep : IMissionStep
{
    [BinaryCount(6)]
    public short[] jointAnglesCentiDeg = new short[6];
    public ushort maxSpeedDegPerSec;
    /// <summary>true = Stuetzstelle (Zwischenpunkt fuer eine gleichmaessige Bewegung, muss nicht
    /// exakt getroffen werden, kein Halt noetig); false = exakte Pose (muss exakt angefahren
    /// werden, Referenz fuers spaetere Blending in roarm_motion.hh -- die eigentliche
    /// Blend-Bewegungslogik ist noch nicht implementiert, dieses Feld transportiert vorerst nur
    /// die Nutzer-Absicht).</summary>
    public bool isWaypoint;
}

/// <summary>Schaltet einen Eintrag aus der Mission-GPIO-Tabelle (siehe roarm_mission_gpio.hh,
/// gpioId indiziert dort hinein) auf den angegebenen Zustand.</summary>
[BinaryType]
public class GpioStep : IMissionStep
{
    public byte gpioId;
    public bool state;
}

/// <summary>Wartet die angegebene Dauer, bevor der naechste Schritt beginnt.</summary>
[BinaryType]
public class DelayStep : IMissionStep
{
    public ushort durationMs;
}

/// <summary>Wire-/Flash-Format einer vollstaendigen Mission -- exakt diese encodierten Bytes legt
/// roarm_mission_store als Dateiinhalt in littlefs ab (der 4-Byte-Nachrichtenkopf dient zugleich
/// als Datei-Magic, kein separates Format noetig).</summary>
[BinaryMessage(MessageKind.Event)]
public class Mission
{
    // Bounds match RoArmSetupAndLoop::kMaxNameLength / kMaxStepsPerMission (roarm.hh) -- this is
    // also the on-flash littlefs file format, so these bounds are load-bearing for existing files,
    // not just wire traffic.
    [BinaryMaxEncodedByteLength(31)] public string name = "";
    [BinaryMaxItemCount(64)] public IMissionStep[] steps = System.Array.Empty<IMissionStep>();
}
