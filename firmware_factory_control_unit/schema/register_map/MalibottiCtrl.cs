using UniversalRegisterAccess;

[ModbusRegion("Malibotti Steuerung")]
public class MalibottiCtrl
{
    /// <summary>Mission-Start/-Status wie RoArmMissionControl: &gt;0 = Mission mit dieser Nummer starten
    /// bzw. sie laeuft, 0 = Leerlauf/letzter Lauf erfolgreich, &lt;0 = Leerlauf/letzter Lauf mit Fehlercode
    /// -Wert (-1 Roboter antwortet nicht, -2 Mission unbekannt, -3 abgebrochen, sonst Fehler des
    /// Roboters). Ein Schreiben wird nur angenommen, wenn der Wert &lt;=0 ist; waehrend einer Mission
    /// schreibt die Firmware die laufende Nummer zurueck. Nur Hardware-Revision 3 (USART6, J16).</summary>
    [ModbusRegister(Access.ReadWrite, Kind = RegisterKind.Command), ModbusMapping(Address = 111)]
    public short MalibottiMissionControl;
}
