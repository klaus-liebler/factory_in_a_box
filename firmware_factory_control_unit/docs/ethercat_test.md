Flashen, sobald die Platine an der Debug-Probe hängt:


dotnet run --project builder -- FlashFirmware --preset rev1
Im Log solltest du danach diese Zeilen sehen:

ethercat: SPI EtherCAT Direct Mapped (system CSRs at 0x3000), falls deine Drähte an Pin 42 und 45 dran sind
usb: … OUT packets waited for COUNT_RX …, sobald USB-Netzwerk läuft; daran erkennst du, wie oft der neue USB-Empfang tatsächlich warten musste
Dann den EtherCAT-Test von der PC-Seite starten, mit dem Kabel an Port A:


python tools/ethercat_master.py --iface "<Netzwerkkarte>" test
Committet ist weiterhin nichts von Firmware und Builder-Bibliothek; das mache ich erst, wenn es auf der Platine läuft.