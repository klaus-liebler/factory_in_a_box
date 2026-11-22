# EtherCAT auf der Platine testen

Flashen (Revision 1 oder 3), sobald die Platine an der Debug-Probe hängt:

```
dotnet run --project builder -- FlashFirmware --preset rev1
```

Im Log (Terminal-Seite oder serielle Konsole) erscheinen danach:

```
ethercat: LAN9253 on SPI, Direct Mapped - ID_REV ..., HW_CFG ...
ethercat: ready - ESC type 0xc0 ..., PDI 0x82, 2 PHYs, polled   (Revision 3: IRQ)
ethercat: DL status 0x.... - PDI ok, port 0 link up ...
```

Die Systemseite der Web-UI zeigt unter "EtherCAT - Slave/Ports/SyncManager und FMMUs" alles, was der ESC
weiß (AL-Zustand, Statuscode, Fehlerzähler je Port, PHYs, Mailbox- und SDO-Zähler, PDO-Größen).

Vom PC aus, Kabel an Port A (IN), Npcap installiert (`pip install scapy`):

```
python tools/ethercat_master.py --list
python tools/ethercat_master.py --iface "<Netzwerkkarte>" test        # alles: SII, AL-Prüfungen, CoE, PDO, OP, Watchdog
python tools/ethercat_master.py --iface "<Netzwerkkarte>" od          # Objektverzeichnis mit Namen
python tools/ethercat_master.py --iface "<Netzwerkkarte>" sdo read 0x1018 2
python tools/ethercat_master.py --iface "<Netzwerkkarte>" sdo write 0x7064 1 3   # Streifen grün
python tools/ethercat_master.py --iface "<Netzwerkkarte>" blink
```

Objektindizes: Eingänge 6000h + Modbus-Adresse des ersten Registers der Region, Ausgänge 7000h + ...,
Parameter 8000h + ..., Kommandos 2000h + ... (`docs/generated/modbus-registers.md`); PDOs 1A00h/1600h +
dieselbe Adresse.

`test` fährt in OP zyklische Prozessdaten (LWR/LRD), setzt das Streifenmuster über das RxPDO (grün,
per SDO zurückgelesen) und hört dann auf: der SyncManager-Watchdog muss die Ausgänge auf 0 setzen
(SAFE-OP mit Fehler 0x001B).

Mit TwinCAT: die ESI `generated/FactoryControlUnit.xml` (vom Builder erzeugt, Schritt
`GenerateRegisterAccessFiles`) nach `C:\TwinCAT\3.1\Config\Io\EtherCAT` kopieren, TwinCAT neu laden, "Scan".
Vendor-ID 0 ist eine Labor-ID - für den echten Einsatz eine ETG-Mitgliedschaft (kostenlos) und eine eigene
Vendor-ID beantragen und in `builder/Program.cs` eintragen.
