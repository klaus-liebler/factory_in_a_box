<!-- GENERIERT von UniversalRegisterAccess -- nicht von Hand editieren. Aenderungen gehoeren in die jeweilige Schema-*.cs-Datei, anschliessend die Codegenerierung erneut ausfuehren. -->

# Modbus-Register

Input-Register (nur lesen): Funktionscode 04. Holding-Register (lesen/schreiben): Funktionscodes 03/06/16. Adressen sind 0-basiert (PDU-Adresse). Mehrwort-Werte belegen aufeinanderfolgende Register[^byteorder].

| Bank | Anzahl Register-Definitionen | Adressbereich |
|---|---|---|
| Input | 58 | 0..257 |
| Holding | 17 | 20..110 |

## Nach Gruppe

### CAN-Bus

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `CAN_TX_COUNT` | Input (R) | 50..51 | 2 | uint32[^byteorder] |  |  |  | decimal |  | ns=1;i=5000 | Gesendete CAN-Frames |
| `CAN_RX_COUNT` | Input (R) | 52..53 | 2 | uint32[^byteorder] |  |  |  | decimal |  | ns=1;i=5001 | Empfangene CAN-Frames |
| `CAN_ERROR_COUNT` | Input (R) | 54..55 | 2 | uint32[^byteorder] |  |  |  | decimal |  | ns=1;i=5002 | CAN-Fehlerzaehler |
| `CAN_LAST_ERROR` | Input (R) | 56 | 1 | uint16 |  |  |  | decimal |  | ns=1;i=5003 | Letzter CAN-Fehlercode |
| `CAN_BUS_STATE` | Input (R) | 57 | 1 | uint16 |  |  |  | decimal |  | ns=1;i=5004 | CAN-Bus-Zustand |

### Farbsensor

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `COLOR_CLEAR` | Input (R) | 160 | 1 | uint16 |  |  |  | decimal | I2C_1 (IRQ PH0) | ns=1;i=5005 | Farbsensor TCS34725, Clear-Kanal |
| `COLOR_RED` | Input (R) | 161 | 1 | uint16 |  |  |  | decimal | I2C_1 (IRQ PH0) | ns=1;i=5006 | Farbsensor TCS34725, Rot-Kanal |
| `COLOR_GREEN` | Input (R) | 162 | 1 | uint16 |  |  |  | decimal | I2C_1 (IRQ PH0) | ns=1;i=5007 | Farbsensor TCS34725, Gruen-Kanal |
| `COLOR_BLUE` | Input (R) | 163 | 1 | uint16 |  |  |  | decimal | I2C_1 (IRQ PH0) | ns=1;i=5008 | Farbsensor TCS34725, Blau-Kanal |

### Diagnostik

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `HEALTH_STATE` | Input (R) | 0 | 1 | Bitfeld (HealthBits) |  |  |  | binary |  | ns=1;i=5009 | Health-Bitfeld (siehe HealthBits) |
| `CHIP_ID` | Input (R) | 1..6 | 6 | Struct (Uid96)[^byteorder] |  |  |  | hex |  | ns=1;i=5010 | 96-Bit Chip-UID |
| `FW_VERSION_MAJOR` | Input (R) | 7 | 1 | uint16 |  |  |  | decimal |  | ns=1;i=5011 | Firmware-Version Major |
| `FW_VERSION_MINOR` | Input (R) | 8 | 1 | uint16 |  |  |  | decimal |  | ns=1;i=5012 | Firmware-Version Minor |
| `FW_VERSION_PATCH` | Input (R) | 9 | 1 | uint16 |  |  |  | decimal |  | ns=1;i=5013 | Firmware-Version Patch |
| `TIMER_TICK` | Input (R) | 10 | 1 | uint16 | ticks |  |  | decimal |  | ns=1;i=5014 | Freilaufender Systick-Zaehler, Ueberlauf nach 65535 |
| `FREE_HEAP_KIB` | Input (R) | 11 | 1 | uint16 | KiB |  |  | decimal |  | ns=1;i=5015 | Freier newlib-Heap (High-Water-Mark bis zur MSP-Stack-Reserve, s. sysmem.c) -- freigegebene, aber noch sbrk'te Bloecke zaehlen nicht mit, also eine pessimistische untere Schranke |
| `CPU_TEMPERATURE_C` | Input (R) | 12 | 1 | int16 | C |  |  | decimal |  | ns=1;i=5016 | Chip-Temperatur (interner DTS-Sensor, RM0481 Kap. 29, werksseitig je Chip kalibriert) |

### Ethernet

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `ETH_LINK_STATUS` | Input (R) | 80 | 1 | uint16 |  |  |  | bool |  | ns=1;i=5017 | 0 = down, 1 = up |
| `ETH_LINK_SPEED` | Input (R) | 81 | 1 | uint16 | Mbit/s |  |  | decimal |  | ns=1;i=5018 | Verbindungsgeschwindigkeit (0 = kein Link) |
| `ETH_LINK_DUPLEX` | Input (R) | 82 | 1 | uint16 |  |  |  | bool |  | ns=1;i=5019 | 0=half, 1=full |
| `ETH_TX_COUNT` | Input (R) | 83..84 | 2 | uint32[^byteorder] |  |  |  | decimal |  | ns=1;i=5020 | Gesendete Ethernet-Frames |
| `ETH_RX_COUNT` | Input (R) | 85..86 | 2 | uint32[^byteorder] |  |  |  | decimal |  | ns=1;i=5021 | Empfangene Ethernet-Frames |
| `ETH_ERROR_COUNT` | Input (R) | 87..88 | 2 | uint32[^byteorder] |  |  |  | decimal |  | ns=1;i=5022 | Ethernet-Fehlerzaehler |
| `ETH_LAST_ERROR` | Input (R) | 89 | 1 | uint16 |  |  |  | decimal |  | ns=1;i=5023 | Letzter Ethernet/PHY-Fehlercode |

### Lichtschranken

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `LIGHTBARRIER1` | Input (R) | 180 | 1 | uint16 |  |  |  | bool | PC6 | ns=1;i=5024 | Lichtschranke 1 (NPN, aktiv=LOW) |
| `LIGHTBARRIER2` | Input (R) | 181 | 1 | uint16 |  |  |  | bool | PC7 | ns=1;i=5025 | Lichtschranke 2 (NPN, aktiv=LOW) |
| `LIGHTBARRIER3` | Input (R) | 182 | 1 | uint16 |  |  |  | bool | PC8 | ns=1;i=5026 | Lichtschranke 3 (NPN, aktiv=LOW) -- nur auf realem Board verdrahtet |

### Motor-PWM

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `CONVEYOR_PWM` | Holding (R/W) | 40 | 1 | uint16 | Promille | 0 | 1000 | range | TIM4_CH3/PD14 | ns=1;i=5027 | Foerderband-Motor Duty |
| `COMPRESSOR_PWM` | Holding (R/W) | 41 | 1 | uint16 | Promille | 0 | 1000 | range | TIM4_CH4/PD15 | ns=1;i=5028 | Kompressor-Motor Duty |

### Stromversorgung

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `PWR_BUS_VOLTAGE_MV` | Input (R) | 110 | 1 | uint16 | mV |  |  | decimal | I2C_4 | ns=1;i=5029 | Busspannung (INA226, 2 mOhm Shunt) |
| `PWR_SHUNT_VOLTAGE_UV` | Input (R) | 111 | 1 | int16 | uV |  |  | decimal | I2C_4 | ns=1;i=5030 | Shunt-Spannung (INA226, 2 mOhm Shunt) |
| `PWR_CURRENT_MA` | Input (R) | 112 | 1 | int16 | mA |  |  | decimal | I2C_4 | ns=1;i=5031 | Stromaufnahme (INA226, 2 mOhm Shunt) |
| `PWR_POWER_MW` | Input (R) | 113 | 1 | uint16 | mW |  |  | decimal | I2C_4 | ns=1;i=5032 | Leistungsaufnahme (INA226) |
| `PWR_PD_VOLTAGE_MV` | Input (R) | 114 | 1 | uint16 | mV |  |  | decimal |  | ns=1;i=5033 | Aktiv ausgehandelte USB-PD-Spannung, PDSink::activeVoltage |
| `PWR_PD_CURRENT_MA` | Input (R) | 115 | 1 | uint16 | mA |  |  | decimal |  | ns=1;i=5034 | Aktiv ausgehandelter USB-PD-Maximalstrom, PDSink::activeCurrent |
| `PWR_PD_STATUS` | Input (R) | 116 | 1 | uint16 |  |  |  | status |  | ns=1;i=5035 | 0 = verbunden und Spannung eingestellt, 1 = verbunden aber Spannung nicht einstellbar, 2 = kein USB-PD-Netzteil erkannt |
| `PWR_STATUS` | Input (R) | 117 | 1 | uint16 |  |  |  | status | I2C_4 | ns=1;i=5036 | 0 = ok, ungleich 0 = Fehler (aktuell: 1 = INA226 nicht erkannt/Init fehlgeschlagen) |

### Drucksensor

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `PRESSURE_RAW` | Input (R) | 200 | 1 | uint16 |  |  |  | decimal | PA4 | ns=1;i=5037 | Analoger Drucksensor, Rohwert (ADC1 CH10) |

### RoArm-M3 Steuerung

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `RO_ARM_MISSION_CONTROL` | Holding (R/W) | 110 | 1 | int16 |  |  |  | decimal |  | ns=1;i=5038 | Mission-Start/-Status: >0 = Mission mit diesem Index laeuft, 0 = Leerlauf/letzter Lauf erfolgreich, <0 = Leerlauf/letzter Lauf mit Fehlercode -Wert. Ein Schreiben durch den Client wird nur uebernommen, wenn der aktuelle Wert <=0 ist; versucht ein Client waehrend einer laufenden Mission zu ueberschreiben, schreibt die Firmware im naechsten Zyklus (<=50ms) den tatsaechlich laufenden Index zurueck (siehe RoArmSetupAndLoop::Loop()). |

### RoArm-M3 Status

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `STATUS` | Input (R) | 250 | 1 | Bitfeld (RoArmStatusBits) |  |  |  | binary |  | ns=1;i=5039 | Status-Bitfeld (siehe RoArmStatusBits) |
| `RO_ARM_ACTIVE_MISSION` | Input (R) | 251 | 1 | int16 |  |  |  | decimal |  | ns=1;i=5040 | Spiegel von RoArmMissionControl fuer lesende OPC-UA-/Modbus-Clients |
| `RO_ARM_JOINT1_ANGLE_CENTI_DEG` | Input (R) | 252 | 1 | int16 | 0.01deg |  |  | decimal |  | ns=1;i=5041 | Aktueller Winkel Gelenk 1 (Base) |
| `RO_ARM_JOINT2_ANGLE_CENTI_DEG` | Input (R) | 253 | 1 | int16 | 0.01deg |  |  | decimal |  | ns=1;i=5042 | Aktueller Winkel Gelenk 2 (Shoulder) |
| `RO_ARM_JOINT3_ANGLE_CENTI_DEG` | Input (R) | 254 | 1 | int16 | 0.01deg |  |  | decimal |  | ns=1;i=5043 | Aktueller Winkel Gelenk 3 (Elbow) |
| `RO_ARM_JOINT4_ANGLE_CENTI_DEG` | Input (R) | 255 | 1 | int16 | 0.01deg |  |  | decimal |  | ns=1;i=5044 | Aktueller Winkel Gelenk 4 (Wrist) |
| `RO_ARM_JOINT5_ANGLE_CENTI_DEG` | Input (R) | 256 | 1 | int16 | 0.01deg |  |  | decimal |  | ns=1;i=5045 | Aktueller Winkel Gelenk 5 (Roll) |
| `RO_ARM_JOINT6_ANGLE_CENTI_DEG` | Input (R) | 257 | 1 | int16 | 0.01deg |  |  | decimal |  | ns=1;i=5046 | Aktueller Winkel Gelenk 6 (Gripper) |

### Waegezelle

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `SCALE_A_WEIGHT` | Input (R) | 220..221 | 2 | int32[^byteorder] |  |  |  | decimal |  | ns=1;i=5047 | Gewicht Waegezelle A |
| `SCALE_A_STATUS` | Input (R) | 222 | 1 | uint16 |  |  |  | status |  | ns=1;i=5048 | 0 = ok, ungleich 0 = Fehler (aktuell: 1 = keine gueltige Messung) |
| `SCALE_B_RAW` | Input (R) | 223..224 | 2 | uint32[^byteorder] |  |  |  | decimal |  | ns=1;i=5049 | Rohwert Waegezelle B (Reserve/Erweiterung) |
| `SCALE_B_STATUS` | Input (R) | 225 | 1 | uint16 |  |  |  | status |  | ns=1;i=5050 | 0 = ok, ungleich 0 = Fehler -- auf dieser Platinen-Revision nicht bestueckt, Register bleibt ungeschrieben (zeigt daher immer 0) |

### Stepper-Steuerung

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `STEPPER_ENABLE` | Holding (R/W) | 60 | 1 | uint16 |  |  |  | bool | PB7 | ns=1;i=5051 | Stepper-Treiber global freigeben |
| `STEPPER1_TARGET` | Holding (R/W) | 61..62 | 2 | int32[^byteorder] |  |  |  | decimal |  | ns=1;i=5052 | Zielposition Stepper 1 |
| `STEPPER1_SPEED` | Holding (R/W) | 63 | 1 | uint16 | Schritte/s |  |  | decimal |  | ns=1;i=5053 | Geschwindigkeit Stepper 1 |
| `STEPPER1_ACCEL` | Holding (R/W) | 64 | 1 | uint16 | Schritte/s^2 |  |  | decimal |  | ns=1;i=5054 | Beschleunigung Stepper 1 |
| `STEPPER2_TARGET` | Holding (R/W) | 65..66 | 2 | int32[^byteorder] |  |  |  | decimal |  | ns=1;i=5055 | Zielposition Stepper 2 |
| `STEPPER2_SPEED` | Holding (R/W) | 67 | 1 | uint16 | Schritte/s |  |  | decimal |  | ns=1;i=5056 | Geschwindigkeit Stepper 2 |
| `STEPPER2_ACCEL` | Holding (R/W) | 68 | 1 | uint16 | Schritte/s^2 |  |  | decimal |  | ns=1;i=5057 | Beschleunigung Stepper 2 |
| `STEPPER1_START_HOMING` | Holding (R/W) | 69 | 1 | uint16 |  | 0 | 255 | decimal |  | ns=1;i=5058 | 1=Homing starten (stoppt Stepper1, sensorloses Homing per StallGuard), 0=erfolgreich beendet, >1=Fehlercode |
| `STEPPER2_START_HOMING` | Holding (R/W) | 70 | 1 | uint16 |  | 0 | 255 | decimal |  | ns=1;i=5059 | 1=Homing starten (stoppt Stepper2, sensorloses Homing per StallGuard), 0=erfolgreich beendet, >1=Fehlercode |

### Stepper-Status

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `STEPPER1_POSITION` | Input (R) | 240..241 | 2 | int32[^byteorder] |  |  |  | decimal |  | ns=1;i=5060 | Aktuelle Position Stepper 1 |
| `STEPPER1_STATUS` | Input (R) | 242 | 1 | uint16 |  |  |  | binary |  | ns=1;i=5061 | Bit0 Enabled, Bit1 Moving, Bit2 Error/StallGuard, Bit3 InPosition |
| `STEPPER2_POSITION` | Input (R) | 243..244 | 2 | int32[^byteorder] |  |  |  | decimal |  | ns=1;i=5062 | Aktuelle Position Stepper 2 |
| `STEPPER2_STATUS` | Input (R) | 245 | 1 | uint16 |  |  |  | binary |  | ns=1;i=5063 | Bit0 Enabled, Bit1 Moving, Bit2 Error/StallGuard, Bit3 InPosition |

### ToF-Sensoren

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `TOF1_DISTANCE_MM` | Input (R) | 140 | 1 | uint16 | mm |  |  | decimal | I2C_1 (IRQ PC15) | ns=1;i=5064 | Abstand Sensor 1 (VL53L0X) |
| `TOF1_STATUS` | Input (R) | 141 | 1 | uint16 |  |  |  | status | I2C_1 (IRQ PC15) | ns=1;i=5065 | 0 = ok, ungleich 0 = Fehler (aktuell: 1 = nicht erkannt/keine gueltige Messung) |
| `TOF2_DISTANCE_MM` | Input (R) | 142 | 1 | uint16 | mm |  |  | decimal | I2C_2 (IRQ PH1) | ns=1;i=5066 | Abstand Sensor 2 (VL53L0X) |
| `TOF2_STATUS` | Input (R) | 143 | 1 | uint16 |  |  |  | status | I2C_2 (IRQ PH1) | ns=1;i=5067 | 0 = ok, ungleich 0 = Fehler (aktuell: 1 = nicht erkannt/keine gueltige Messung) |
| `TOF3_DISTANCE_MM` | Input (R) | 144 | 1 | uint16 | mm |  |  | decimal | I2C_4 (IRQ PA3) | ns=1;i=5068 | Abstand Sensor 3 (VL53L0X) |
| `TOF3_STATUS` | Input (R) | 145 | 1 | uint16 |  |  |  | status | I2C_4 (IRQ PA3) | ns=1;i=5069 | 0 = ok, ungleich 0 = Fehler (aktuell: 1 = nicht erkannt/keine gueltige Messung) |

### Pneumatikventile

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `VALVE1` | Holding (R/W) | 20 | 1 | uint16 |  |  |  | bool | PD10 | ns=1;i=5070 | Pneumatikventil 1 |
| `VALVE2` | Holding (R/W) | 21 | 1 | uint16 |  |  |  | bool | PD5 | ns=1;i=5071 | Pneumatikventil 2 |
| `VALVE3` | Holding (R/W) | 22 | 1 | uint16 |  |  |  | bool | PD11 | ns=1;i=5072 | Pneumatikventil 3 |

### WS2812

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `WS2812_CH1_PATTERN` | Holding (R/W) | 100 | 1 | uint16 |  |  |  | decimal | PE5/TIM15_CH1 | ns=1;i=5073 | Index in fest im Code hinterlegte Farbmuster-Tabelle |
| `WS2812_CH2_PATTERN` | Holding (R/W) | 101 | 1 | uint16 |  |  |  | decimal | PE6/TIM15_CH2 | ns=1;i=5074 | Index in fest im Code hinterlegte Farbmuster-Tabelle |

## Input-Register nach Adresse (FC04)

| Adresse | Worte | Gruppe | Bezeichnung | Typ | Einheit |
|---:|---:|---|---|---|---|
| 0 | 1 | Diagnostik | `HEALTH_STATE` | Bitfeld (HealthBits) |  |
| 1..6 | 6 | Diagnostik | `CHIP_ID` | Struct (Uid96)[^byteorder] |  |
| 7 | 1 | Diagnostik | `FW_VERSION_MAJOR` | uint16 |  |
| 8 | 1 | Diagnostik | `FW_VERSION_MINOR` | uint16 |  |
| 9 | 1 | Diagnostik | `FW_VERSION_PATCH` | uint16 |  |
| 10 | 1 | Diagnostik | `TIMER_TICK` | uint16 | ticks |
| 11 | 1 | Diagnostik | `FREE_HEAP_KIB` | uint16 | KiB |
| 12 | 1 | Diagnostik | `CPU_TEMPERATURE_C` | int16 | C |
| 50..51 | 2 | CAN-Bus | `CAN_TX_COUNT` | uint32[^byteorder] |  |
| 52..53 | 2 | CAN-Bus | `CAN_RX_COUNT` | uint32[^byteorder] |  |
| 54..55 | 2 | CAN-Bus | `CAN_ERROR_COUNT` | uint32[^byteorder] |  |
| 56 | 1 | CAN-Bus | `CAN_LAST_ERROR` | uint16 |  |
| 57 | 1 | CAN-Bus | `CAN_BUS_STATE` | uint16 |  |
| 80 | 1 | Ethernet | `ETH_LINK_STATUS` | uint16 |  |
| 81 | 1 | Ethernet | `ETH_LINK_SPEED` | uint16 | Mbit/s |
| 82 | 1 | Ethernet | `ETH_LINK_DUPLEX` | uint16 |  |
| 83..84 | 2 | Ethernet | `ETH_TX_COUNT` | uint32[^byteorder] |  |
| 85..86 | 2 | Ethernet | `ETH_RX_COUNT` | uint32[^byteorder] |  |
| 87..88 | 2 | Ethernet | `ETH_ERROR_COUNT` | uint32[^byteorder] |  |
| 89 | 1 | Ethernet | `ETH_LAST_ERROR` | uint16 |  |
| 110 | 1 | Stromversorgung | `PWR_BUS_VOLTAGE_MV` | uint16 | mV |
| 111 | 1 | Stromversorgung | `PWR_SHUNT_VOLTAGE_UV` | int16 | uV |
| 112 | 1 | Stromversorgung | `PWR_CURRENT_MA` | int16 | mA |
| 113 | 1 | Stromversorgung | `PWR_POWER_MW` | uint16 | mW |
| 114 | 1 | Stromversorgung | `PWR_PD_VOLTAGE_MV` | uint16 | mV |
| 115 | 1 | Stromversorgung | `PWR_PD_CURRENT_MA` | uint16 | mA |
| 116 | 1 | Stromversorgung | `PWR_PD_STATUS` | uint16 |  |
| 117 | 1 | Stromversorgung | `PWR_STATUS` | uint16 |  |
| 140 | 1 | ToF-Sensoren | `TOF1_DISTANCE_MM` | uint16 | mm |
| 141 | 1 | ToF-Sensoren | `TOF1_STATUS` | uint16 |  |
| 142 | 1 | ToF-Sensoren | `TOF2_DISTANCE_MM` | uint16 | mm |
| 143 | 1 | ToF-Sensoren | `TOF2_STATUS` | uint16 |  |
| 144 | 1 | ToF-Sensoren | `TOF3_DISTANCE_MM` | uint16 | mm |
| 145 | 1 | ToF-Sensoren | `TOF3_STATUS` | uint16 |  |
| 160 | 1 | Farbsensor | `COLOR_CLEAR` | uint16 |  |
| 161 | 1 | Farbsensor | `COLOR_RED` | uint16 |  |
| 162 | 1 | Farbsensor | `COLOR_GREEN` | uint16 |  |
| 163 | 1 | Farbsensor | `COLOR_BLUE` | uint16 |  |
| 180 | 1 | Lichtschranken | `LIGHTBARRIER1` | uint16 |  |
| 181 | 1 | Lichtschranken | `LIGHTBARRIER2` | uint16 |  |
| 182 | 1 | Lichtschranken | `LIGHTBARRIER3` | uint16 |  |
| 200 | 1 | Drucksensor | `PRESSURE_RAW` | uint16 |  |
| 220..221 | 2 | Waegezelle | `SCALE_A_WEIGHT` | int32[^byteorder] |  |
| 222 | 1 | Waegezelle | `SCALE_A_STATUS` | uint16 |  |
| 223..224 | 2 | Waegezelle | `SCALE_B_RAW` | uint32[^byteorder] |  |
| 225 | 1 | Waegezelle | `SCALE_B_STATUS` | uint16 |  |
| 240..241 | 2 | Stepper-Status | `STEPPER1_POSITION` | int32[^byteorder] |  |
| 242 | 1 | Stepper-Status | `STEPPER1_STATUS` | uint16 |  |
| 243..244 | 2 | Stepper-Status | `STEPPER2_POSITION` | int32[^byteorder] |  |
| 245 | 1 | Stepper-Status | `STEPPER2_STATUS` | uint16 |  |
| 250 | 1 | RoArm-M3 Status | `STATUS` | Bitfeld (RoArmStatusBits) |  |
| 251 | 1 | RoArm-M3 Status | `RO_ARM_ACTIVE_MISSION` | int16 |  |
| 252 | 1 | RoArm-M3 Status | `RO_ARM_JOINT1_ANGLE_CENTI_DEG` | int16 | 0.01deg |
| 253 | 1 | RoArm-M3 Status | `RO_ARM_JOINT2_ANGLE_CENTI_DEG` | int16 | 0.01deg |
| 254 | 1 | RoArm-M3 Status | `RO_ARM_JOINT3_ANGLE_CENTI_DEG` | int16 | 0.01deg |
| 255 | 1 | RoArm-M3 Status | `RO_ARM_JOINT4_ANGLE_CENTI_DEG` | int16 | 0.01deg |
| 256 | 1 | RoArm-M3 Status | `RO_ARM_JOINT5_ANGLE_CENTI_DEG` | int16 | 0.01deg |
| 257 | 1 | RoArm-M3 Status | `RO_ARM_JOINT6_ANGLE_CENTI_DEG` | int16 | 0.01deg |

## Holding-Register nach Adresse (FC03/06/16)

| Adresse | Worte | Gruppe | Bezeichnung | Typ | Einheit |
|---:|---:|---|---|---|---|
| 20 | 1 | Pneumatikventile | `VALVE1` | uint16 |  |
| 21 | 1 | Pneumatikventile | `VALVE2` | uint16 |  |
| 22 | 1 | Pneumatikventile | `VALVE3` | uint16 |  |
| 40 | 1 | Motor-PWM | `CONVEYOR_PWM` | uint16 | Promille |
| 41 | 1 | Motor-PWM | `COMPRESSOR_PWM` | uint16 | Promille |
| 60 | 1 | Stepper-Steuerung | `STEPPER_ENABLE` | uint16 |  |
| 61..62 | 2 | Stepper-Steuerung | `STEPPER1_TARGET` | int32[^byteorder] |  |
| 63 | 1 | Stepper-Steuerung | `STEPPER1_SPEED` | uint16 | Schritte/s |
| 64 | 1 | Stepper-Steuerung | `STEPPER1_ACCEL` | uint16 | Schritte/s^2 |
| 65..66 | 2 | Stepper-Steuerung | `STEPPER2_TARGET` | int32[^byteorder] |  |
| 67 | 1 | Stepper-Steuerung | `STEPPER2_SPEED` | uint16 | Schritte/s |
| 68 | 1 | Stepper-Steuerung | `STEPPER2_ACCEL` | uint16 | Schritte/s^2 |
| 69 | 1 | Stepper-Steuerung | `STEPPER1_START_HOMING` | uint16 |  |
| 70 | 1 | Stepper-Steuerung | `STEPPER2_START_HOMING` | uint16 |  |
| 100 | 1 | WS2812 | `WS2812_CH1_PATTERN` | uint16 |  |
| 101 | 1 | WS2812 | `WS2812_CH2_PATTERN` | uint16 |  |
| 110 | 1 | RoArm-M3 Steuerung | `RO_ARM_MISSION_CONTROL` | int16 |  |

## Bitfeld-Typen

### HealthBits

Health-Bitfeld (siehe HealthState) -- Bit-Positionen, keine Registerwerte.

| Bit | Name |
|---:|---|
| 0 | `Ok` |
| 1 | `Overtemperature` |
| 2 | `PowerFault` |
| 3 | `CanError` |
| 4 | `EthLinkDown` |
| 5 | `I2cSensorFault` |
| 6 | `StepperFault` |

### RoArmStatusBits

RoArm-Status-Bitfeld (siehe RoArmStatus.Status) -- Bit-Positionen, keine Registerwerte.

| Bit | Name |
|---:|---|
| 0 | `TorqueEnabled` |
| 1 | `Moving` |
| 2 | `Error` |
| 3 | `TeachModeActive` |

[^byteorder]: Byte-Order durchgehend Big-Endian ("ABCD"): Jedes Register wird High-Byte zuerst uebertragen (Modbus-Standard), und bei 32-Bit-Werten liegt das hoeherwertige Wort (High-Word) auf der niedrigeren Adresse, das niederwertige Wort (Low-Word) auf der Adresse +1. Vorzeichenbehaftete Werte (int32) sind im Zweierkomplement codiert. Structs bestehen aus 32-Bit-Feldern in Deklarationsreihenfolge, jedes Feld ebenfalls High-Word zuerst.

    Beispiel: `SCALE_A_WEIGHT` (int32, Input-Register 220..221) mit dem Wert -100000 = 0xFFFE7960 (Zweierkomplement):

    - Register 220 (High-Word) = 0xFFFE, Register 221 (Low-Word) = 0x7960
    - Datenbytes in der FC04-Antwort (ab Startadresse 220, Anzahl 2): `FF FE 79 60`
    - Zusammensetzen: `(0xFFFE << 16) | 0x7960` = 0xFFFE7960, als int32 interpretiert -100000
    - Bei Clients mit einstellbarer Wort-Reihenfolge (z. B. "Word Swap"/"CDAB") also KEINEN Wort-Tausch waehlen.
