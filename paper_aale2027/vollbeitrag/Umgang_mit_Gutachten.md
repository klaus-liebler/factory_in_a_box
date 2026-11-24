# AALE 2027, Beitrag 129 „Factory Control Unit“: Umgang mit den Gutachten

## Gutachten 1 (70 Punkte; Methodik 6, Darstellung 6)
- **Methodik:** Kapitel 4.1 beschreibt das Entwicklungsvorgehen jetzt explizit in vier Grundsätzen (Best-in-Class-Analyse, C++26 mit Compilezeit-Prüfungen, Errata-Abgleich, Testkette). Kapitel 4.2 beschreibt die dreistufige Testkette, Kapitel 4.5 Fuzzing und Penetrationstest mit Zahlen (über 700 Testfälle, 309 portierte open62541-Fälle, rund 437 Mio. AFL++-Ausführungen) und Befunden (Tabelle 1).
- **Darstellung:** klarer roter Faden (Anlage → Hardware → Software/Methode → SPS-Umgebung → Einordnung → Lehre) und ein Architekturbild (Bild 1).

## Gutachten 2 (90 Punkte)
- Lob für CODESYS statt S7 und den herstellerneutralen Ansatz: Kapitel 1 greift das als Begründung auf („Prinzip statt Werkzeug“). Kapitel 5 erweitert es um eine eigene, herstellerneutrale IEC-61131-3-Umgebung im Browser.
- OPC UA als Zukunftsthema: OPC UA jetzt mit Sign **und** SignAndEncrypt, Subscriptions und CA-basierter Client-Prüfung (Kapitel 4.4/4.5).

## Gutachten 3 (70 Punkte; Methodik 5)
- **„Mikrocontroller statt SPS ist unglücklich“:** Kapitel 1, zweiter Absatz: Die FCU ersetzt die SPS nicht. Sie übernimmt die Rolle der dezentralen Peripherie bzw. des intelligenten Feldgeräts und bietet OPC UA, Modbus und EtherCAT für jede (Soft-)SPS. Begründet wird das mit der Sensorik/Aktorik, die an einer Kompakt-SPS viele Zusatzmodule bräuchte, und mit dem Budget unter 1.000 €. Zusätzlich gibt es die eigene IEC-61131-3-Laufzeit.
- **„Gesamtheit wirkt verwirrend, auf einen Teil fokussieren“:** Der Beitrag konzentriert sich jetzt auf die Software und ihre KI-gestützte Entstehung. Mechanik (Kap. 2) und Hardware (Kap. 3) sind auf je einen Absatz gekürzt, die 3D-Missionsplanung und die Details der Druckregelung sind entfallen. Das steht auch so im letzten Absatz von Kapitel 1.
- **Energiemonitoring nur am Rande:** Es ist nicht mehr als Schwerpunkt ausgewiesen. Kapitel 7 sagt ausdrücklich, dass verursachergerechtes Energiemonitoring nicht Gegenstand des Beitrags ist; die Behauptungen zu Hilfsenergie und 0,1 W wurden gestrichen.

## Abweichungen von der Kurzfassung (bitte bewusst entscheiden)
- ThreadX/NetX/USBX ersetzt durch rtoos; C++20 → C++26 (in Kap. 1 offen benannt).
- Untertitel ergänzt: „Modellgetriebene, vollständig KI-generierte Steuerung …“.
- Stichwort „Modellgetriebene Entwicklung“ ersetzt durch „KI-gestützte Softwareentwicklung, IEC 61131-3, OPC UA“.
