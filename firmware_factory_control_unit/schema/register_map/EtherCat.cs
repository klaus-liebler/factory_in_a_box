using UniversalRegisterAccess;

// The EtherCAT slave's state (src/ethercat.cpp, refreshed once a second); everything else the ESC
// knows is on the web UI's system page (WebSocket namespace ethercat).
[ModbusRegion("EtherCAT")]
public class EtherCat
{
    /// <summary>AL-Status: 1 INIT, 2 PRE-OP, 4 SAFE-OP, 8 OP; Bit 4 (0x10) Fehler angezeigt; 0 = kein ESC</summary>
    [ModbusRegister(Access.ReadOnly, Display = "hex"), ModbusMapping(Address = 90)]
    public ushort EcatAlStatus;

    /// <summary>AL-Statuscode (ETG.1000.6), 0 = kein Fehler</summary>
    [ModbusRegister(Access.ReadOnly, Display = "status"), ModbusMapping(Address = 91)]
    public ushort EcatAlStatusCode;

    /// <summary>DL-Status: Bit 0 PDI betriebsbereit, Bit 4/5 Link Port 0/1, Bit 9/11 Kommunikation Port 0/1</summary>
    [ModbusRegister(Access.ReadOnly, Display = "binary"), ModbusMapping(Address = 92)]
    public ushort EcatDlStatus;

    /// <summary>Vom Master vergebene Stationsadresse</summary>
    [ModbusRegister(Access.ReadOnly, Display = "hex"), ModbusMapping(Address = 93)]
    public ushort EcatStationAddress;

    /// <summary>Empfangsfehler beider Ports (ungueltige Frames, RX-Fehler, weitergeleitete Fehler), saettigend</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 94)]
    public ushort EcatRxErrors;

    /// <summary>Link-Verluste beider Ports, saettigend</summary>
    [ModbusRegister(Access.ReadOnly), ModbusMapping(Address = 95)]
    public ushort EcatLostLinks;
}
