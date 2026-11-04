using BestBinaryBuffers;

namespace pneumatics;

/// <summary>Regelrelevante Mess- und Stellwerte der Druckregelstrecke (s.
/// docs/druckregelstrecke-modes.md), periodisch gesendet -- bewusst kompakt (ein einzelnes
/// Registerfeld je Groesse statt des kompletten Modbus-Registersatzes), damit der clientseitige
/// Regler im Reglerbetrieb-Modus (500ms-Zyklus) sie unabhaengig vom schwereren Mehr-Paket-
/// "/api/registers"-Polling der uebrigen Seiten lesen kann.</summary>
[BinaryMessage(MessageKind.Event)]
public class PressureControlFeedback
{
    public ushort pressureRaw;
    public ushort compressorPwmPermille;
    public bool valve1Open;
    public bool valve2Open;
    public bool valve3Open;
}
