using UniversalRegisterAccess;

[ModbusRegion("RoArm-M3 Steuerung")]
public class RoArmCtrl
{
    /// <summary>Mission-Start/-Status: &gt;0 = Mission mit diesem Index laeuft, 0 = Leerlauf/letzter
    /// Lauf erfolgreich, &lt;0 = Leerlauf/letzter Lauf mit Fehlercode -Wert. Ein Schreiben durch den
    /// Client wird nur uebernommen, wenn der aktuelle Wert &lt;=0 ist; versucht ein Client waehrend
    /// einer laufenden Mission zu ueberschreiben, schreibt die Firmware im naechsten Zyklus
    /// (&lt;=50ms) den tatsaechlich laufenden Index zurueck (siehe RoArmSetupAndLoop::Loop()).</summary>
    [ModbusRegister(Access.ReadWrite), ModbusMapping(Address = 110)]
    public short RoArmMissionControl;
}
