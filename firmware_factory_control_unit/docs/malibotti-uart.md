# Malibotti – UART-Protokoll

Malibotti ist ein einfacher Roboter auf ESP32-S3-Basis. Er wird nicht geteacht: Seine Missionen
kennt er selbst (nummeriert ab 1). Das FCU sagt ihm nur, welche er abarbeiten soll, und meldet
seinen Zustand in Registern. Vorhanden ab **Hardware-Revision 3**. Auf Revision 1 bleiben die
Register 0.

## Verbindung

| | |
|---|---|
| Stecker | J16: 1 = MALIBOTTI_RX (PC7, FCU empfängt), 2 = MALIBOTTI_TX (PC6, FCU sendet), 3 = GND, 4 = +5 V |
| Schnittstelle | USART6, 115200 Bd, 8N1, 3,3-V-Pegel (ESP32-S3: direkt; keine Serienwiderstände auf der Platine) |
| Verkabelung | FCU TX (J16.2) → Roboter RX, Roboter TX → FCU RX (J16.1), GND gemeinsam |

## Rahmen

Ein UART kennt keine Nachrichtengrenzen und verliert oder verfälscht einzelne Bytes. Jede
Nachricht wird deshalb so gesendet:

```
COBS( Nachricht | CRC-16 ) 0x00
```

- **Nachricht**: dieselbe BestBinaryBuffers-Kodierung wie auf dem WebSocket, also Namespace-ID
  und Typ-ID (je 2 Byte, little endian), dann die Felder. Das Schema steht in
  `schema/best_binary_buffers/malibotti.cs` (Namespace `malibotti`, ID 9). Der Generator erzeugt
  den C++-Code für beide Seiten: Das ESP-IDF-Projekt des Roboters bindet `ws_protocol.hh`
  ebenfalls ein, oder es generiert mit `--best-binary-buffer-schema-path` aus demselben Schema.
- **CRC-16/CCITT-FALSE** über die Nachricht: Polynom 0x1021, Startwert 0xFFFF, little endian
  angehängt. Prüfwert: „123456789“ → 0x29B1.
- **COBS** (Consistent Overhead Byte Stuffing) entfernt jedes 0x00 aus dem Rahmen, auf 254 Bytes
  kostet das höchstens ein Byte mehr. Damit steht **0x00 nur am Rahmenende**. Ein Empfänger, der
  mitten in einem Rahmen anfängt oder ein Byte verliert, ist am nächsten 0x00 wieder im Takt. Ein
  verfälschter Rahmen fällt an der CRC heraus.

Implementierung: `rtoos/include/rtos/serial/frame.hpp` (`encode_frame`, `FrameDecoder`,
header-only und ohne Allokation, also auch auf dem ESP32 verwendbar). Tests:
`rtoos/tests/serial_frame_test.cpp`.

## Nachrichten

**Roboter → FCU: `Status`** (alle 200 ms und sofort bei jeder Änderung)

| Feld | Typ | Bedeutung |
|---|---|---|
| ackSeq | u8 | seq des letzten übernommenen Kommandos (0 = keines seit dem Start des Roboters) |
| state | RobotState | Idle 0, Busy 1, Error 2 |
| activeMission | u16 | laufende Mission, 0 = keine |
| lastMission | u16 | die Mission, zu der lastResult gehört |
| lastResult | MissionResult | Ok 0, UnknownMission 2, Aborted 3, Busy 4, Blocked 5, Fault 6 |
| supplyMv | u16 | Akku- oder Versorgungsspannung, 0 = unbekannt |

**FCU → Roboter: `Command`**

| Feld | Typ | Bedeutung |
|---|---|---|
| seq | u8 | 1..255, läuft um; 0 kommt nie vor |
| kind | CommandKind | Ping 0 (nur Quittung), StartMission 1, Abort 2 |
| mission | u16 | bei StartMission: die Nummer |

## Ablauf

- Kommt **1 s lang kein Status**, gilt der Roboter als nicht verbunden.
- Ein Kommando bekommt eine neue seq. Das FCU wiederholt es **alle 200 ms**, bis ein Status diese
  seq in `ackSeq` trägt; nach 1 s ohne Quittung gibt es auf. Ein verlorener Rahmen kostet in jeder
  Richtung also nur eine Wiederholung.
- Der Roboter führt jede seq **genau einmal** aus. Ein wiederholtes Kommando mit derselben seq
  beantwortet er nur mit seinem Status, eine laufende Mission startet also nie doppelt. Er
  übernimmt das Kommando, *bevor* er den Status mit dessen ackSeq sendet. Dieser Status zeigt also
  schon die Wirkung (Busy, oder lastResult bei einer Ablehnung).
- Ein Start, während eine Mission läuft: lastMission = die abgelehnte Nummer, lastResult = Busy.

```
FCU                                   Roboter
 | Command seq=7 StartMission 3  -->  |   (verloren)
 | Command seq=7 StartMission 3  -->  |   startet Mission 3
 |  <-- Status ackSeq=7 Busy active=3 |
 |  <-- Status ackSeq=7 Busy active=3 |   alle 200 ms
 |  <-- Status ackSeq=7 Idle last=3 Ok|   fertig
```

## Register

| Register | Art | Bedeutung |
|---|---|---|
| MalibottiMissionControl (Holding 111) | Kommando | wie RoArmMissionControl: >0 schreiben startet diese Mission; während sie läuft, steht ihre Nummer darin (andere Werte schreibt die Firmware zurück, -3 bricht ab). Danach: 0 = erfolgreich, <0 = Fehler: -1 Roboter antwortet nicht, -2 Mission unbekannt, -3 abgebrochen, -4 belegt, -5 blockiert, -6 Störung |
| MalibottiStatus (Input 260) | Bitfeld | Bit 0 verbunden, Bit 1 Mission läuft, Bit 2 Fehler |
| MalibottiActiveMission (Input 261) | | laufende Mission laut Roboter, 0 = keine |

In der SPS: Baustein `FCU_MalibottiRunMission` (FCU-Bibliothek, wie `FCU_RoArmRunMission`).
Die Firmware-Seite: `src/malibotti.cpp` (im I/O-Zyklus, alle 50 ms).

## Was der Roboter tun muss

1. UART 115200 8N1, Bytes in einen `FrameDecoder` schieben.
2. Bei einem `Command`: Ist die seq neu, das Kommando ausführen (StartMission, Abort, Ping).
   Danach `ackSeq = seq` setzen und sofort einen `Status` senden.
3. Alle 200 ms und bei jeder Zustandsänderung einen `Status` senden.
