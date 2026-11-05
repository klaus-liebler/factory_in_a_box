<!-- GENERIERT von UniversalRegisterAccess -- nicht von Hand editieren. Aenderungen gehoeren in die jeweilige Schema-*.cs-Datei, anschliessend die Codegenerierung erneut ausfuehren. -->

# Modbus-Register

Input-Register (nur lesen): Funktionscode 04. Holding-Register (lesen/schreiben): Funktionscodes 03/06/16. Adressen sind 0-basiert (PDU-Adresse). Mehrwort-Werte belegen aufeinanderfolgende Register[^byteorder].

| Bank | Anzahl Register-Definitionen | Adressbereich |
|---|---|---|
| Input | 58 | 0..257 |
| Holding | 19 | 20..110 |

## Nach Gruppe

### CAN-Bus

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `can_tx_count` | Input (R) | 50..51 | 2 | uint32[^byteorder] |  |  |  | decimal |  | ns=1;i=5000 | Gesendete CAN-Frames |
| `can_rx_count` | Input (R) | 52..53 | 2 | uint32[^byteorder] |  |  |  | decimal |  | ns=1;i=5001 | Empfangene CAN-Frames |
| `can_error_count` | Input (R) | 54..55 | 2 | uint32[^byteorder] |  |  |  | decimal |  | ns=1;i=5002 | CAN-Fehlerzaehler |
| `can_last_error` | Input (R) | 56 | 1 | uint16 |  |  |  | decimal |  | ns=1;i=5003 | Letzter CAN-Fehlercode |
| `can_bus_state` | Input (R) | 57 | 1 | uint16 |  |  |  | decimal |  | ns=1;i=5004 | CAN-Bus-Zustand |

### Farbsensor

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `color_clear` | Input (R) | 160 | 1 | uint16 |  |  |  | decimal | I2C_1 (IRQ Rev1 PH0, Rev2 PH1) | ns=1;i=5005 | Farbsensor TCS34725, Clear-Kanal |
| `color_red` | Input (R) | 161 | 1 | uint16 |  |  |  | decimal | I2C_1 (IRQ Rev1 PH0, Rev2 PH1) | ns=1;i=5006 | Farbsensor TCS34725, Rot-Kanal |
| `color_green` | Input (R) | 162 | 1 | uint16 |  |  |  | decimal | I2C_1 (IRQ Rev1 PH0, Rev2 PH1) | ns=1;i=5007 | Farbsensor TCS34725, Gruen-Kanal |
| `color_blue` | Input (R) | 163 | 1 | uint16 |  |  |  | decimal | I2C_1 (IRQ Rev1 PH0, Rev2 PH1) | ns=1;i=5008 | Farbsensor TCS34725, Blau-Kanal |

### Diagnostik

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `health_state` | Input (R) | 0 | 1 | Bitfeld (HealthBits) |  |  |  | binary |  | ns=1;i=5009 | Health-Bitfeld (siehe HealthBits) |
| `chip_id` | Input (R) | 1..6 | 6 | Struct (Uid96)[^byteorder] |  |  |  | hex |  | ns=1;i=5010 | 96-Bit Chip-UID |
| `fw_version_major` | Input (R) | 7 | 1 | uint16 |  |  |  | decimal |  | ns=1;i=5011 | Firmware-Version Major |
| `fw_version_minor` | Input (R) | 8 | 1 | uint16 |  |  |  | decimal |  | ns=1;i=5012 | Firmware-Version Minor |
| `fw_version_patch` | Input (R) | 9 | 1 | uint16 |  |  |  | decimal |  | ns=1;i=5013 | Firmware-Version Patch |
| `timer_tick` | Input (R) | 10 | 1 | uint16 | ticks |  |  | decimal |  | ns=1;i=5014 | Freilaufender Systick-Zaehler, Ueberlauf nach 65535 |
| `free_heap_kib` | Input (R) | 11 | 1 | uint16 | KiB |  |  | decimal |  | ns=1;i=5015 | Freier newlib-Heap (High-Water-Mark bis zur MSP-Stack-Reserve, s. sysmem.c) -- freigegebene, aber noch sbrk'te Bloecke zaehlen nicht mit, also eine pessimistische untere Schranke |
| `cpu_temperature_c` | Input (R) | 12 | 1 | int16 | C |  |  | decimal |  | ns=1;i=5016 | Chip-Temperatur (interner DTS-Sensor, RM0481 Kap. 29, werksseitig je Chip kalibriert) |

### Ethernet

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `eth_link_status` | Input (R) | 80 | 1 | uint16 |  |  |  | bool |  | ns=1;i=5017 | 0 = down, 1 = up |
| `eth_link_speed` | Input (R) | 81 | 1 | uint16 | Mbit/s |  |  | decimal |  | ns=1;i=5018 | Verbindungsgeschwindigkeit (0 = kein Link) |
| `eth_link_duplex` | Input (R) | 82 | 1 | uint16 |  |  |  | bool |  | ns=1;i=5019 | 0=half, 1=full |
| `eth_tx_count` | Input (R) | 83..84 | 2 | uint32[^byteorder] |  |  |  | decimal |  | ns=1;i=5020 | Gesendete Ethernet-Frames |
| `eth_rx_count` | Input (R) | 85..86 | 2 | uint32[^byteorder] |  |  |  | decimal |  | ns=1;i=5021 | Empfangene Ethernet-Frames |
| `eth_error_count` | Input (R) | 87..88 | 2 | uint32[^byteorder] |  |  |  | decimal |  | ns=1;i=5022 | Ethernet-Fehlerzaehler |
| `eth_last_error` | Input (R) | 89 | 1 | uint16 |  |  |  | decimal |  | ns=1;i=5023 | Letzter Ethernet/PHY-Fehlercode |

### Lichtschranken

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `lightbarrier1` | Input (R) | 180 | 1 | uint16 |  |  |  | bool | Rev1 PC6, Rev2 PC7 | ns=1;i=5024 | Lichtschranke 1 (NPN, aktiv=LOW) |
| `lightbarrier2` | Input (R) | 181 | 1 | uint16 |  |  |  | bool | Rev1 PC7, Rev2 PC8 | ns=1;i=5025 | Lichtschranke 2 (NPN, aktiv=LOW) |
| `lightbarrier3` | Input (R) | 182 | 1 | uint16 |  |  |  | bool | Rev1 PC8 | ns=1;i=5026 | Lichtschranke 3 (NPN, aktiv=LOW) -- nur Hardware-Revision 1 |

### Motor-PWM

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `conveyor_pwm` | Holding (R/W) | 40 | 1 | uint16 | Promille | 0 | 1000 | range | TIM4_CH3/PD14 | ns=1;i=5027 | Foerderband-Motor Duty |
| `compressor_pwm` | Holding (R/W) | 41 | 1 | uint16 | Promille | 0 | 1000 | range | TIM4_CH4/PD15 | ns=1;i=5028 | Kompressor-Motor Duty |

### Stromversorgung

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `pwr_bus_voltage_mv` | Input (R) | 110 | 1 | uint16 | mV |  |  | decimal | I2C_4 | ns=1;i=5029 | Busspannung (INA226, 2 mOhm Shunt) |
| `pwr_shunt_voltage_uv` | Input (R) | 111 | 1 | int16 | uV |  |  | decimal | I2C_4 | ns=1;i=5030 | Shunt-Spannung (INA226, 2 mOhm Shunt) |
| `pwr_current_ma` | Input (R) | 112 | 1 | int16 | mA |  |  | decimal | I2C_4 | ns=1;i=5031 | Stromaufnahme (INA226, 2 mOhm Shunt) |
| `pwr_power_mw` | Input (R) | 113 | 1 | uint16 | mW |  |  | decimal | I2C_4 | ns=1;i=5032 | Leistungsaufnahme (INA226) |
| `pwr_pd_voltage_mv` | Input (R) | 114 | 1 | uint16 | mV |  |  | decimal |  | ns=1;i=5033 | Aktiv ausgehandelte USB-PD-Spannung, PDSink::activeVoltage |
| `pwr_pd_current_ma` | Input (R) | 115 | 1 | uint16 | mA |  |  | decimal |  | ns=1;i=5034 | Aktiv ausgehandelter USB-PD-Maximalstrom, PDSink::activeCurrent |
| `pwr_pd_status` | Input (R) | 116 | 1 | uint16 |  |  |  | status |  | ns=1;i=5035 | 0 = verbunden und Spannung eingestellt, 1 = verbunden aber Spannung nicht einstellbar, 2 = kein USB-PD-Netzteil erkannt |
| `pwr_status` | Input (R) | 117 | 1 | uint16 |  |  |  | status | I2C_4 | ns=1;i=5036 | 0 = ok, ungleich 0 = Fehler (aktuell: 1 = INA226 nicht erkannt/Init fehlgeschlagen) |

### Drucksensor

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `pressure_raw` | Input (R) | 200 | 1 | uint16 |  |  |  | decimal | PA4 | ns=1;i=5037 | Analoger Drucksensor, Rohwert (ADC1 CH10) |

### RoArm-M3 Steuerung

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `ro_arm_mission_control` | Holding (R/W) | 110 | 1 | int16 |  |  |  | decimal |  | ns=1;i=5038 | Mission-Start/-Status: >0 = Mission mit diesem Index laeuft, 0 = Leerlauf/letzter Lauf erfolgreich, <0 = Leerlauf/letzter Lauf mit Fehlercode -Wert. Ein Schreiben durch den Client wird nur uebernommen, wenn der aktuelle Wert <=0 ist; versucht ein Client waehrend einer laufenden Mission zu ueberschreiben, schreibt die Firmware im naechsten Zyklus (<=50ms) den tatsaechlich laufenden Index zurueck (siehe RoArmSetupAndLoop::Loop()). |

### RoArm-M3 Status

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `status` | Input (R) | 250 | 1 | Bitfeld (RoArmStatusBits) |  |  |  | binary |  | ns=1;i=5039 | Status-Bitfeld (siehe RoArmStatusBits) |
| `ro_arm_active_mission` | Input (R) | 251 | 1 | int16 |  |  |  | decimal |  | ns=1;i=5040 | Spiegel von RoArmMissionControl fuer lesende OPC-UA-/Modbus-Clients |
| `ro_arm_joint1_angle_centi_deg` | Input (R) | 252 | 1 | int16 | 0.01deg |  |  | decimal |  | ns=1;i=5041 | Aktueller Winkel Gelenk 1 (Base) |
| `ro_arm_joint2_angle_centi_deg` | Input (R) | 253 | 1 | int16 | 0.01deg |  |  | decimal |  | ns=1;i=5042 | Aktueller Winkel Gelenk 2 (Shoulder) |
| `ro_arm_joint3_angle_centi_deg` | Input (R) | 254 | 1 | int16 | 0.01deg |  |  | decimal |  | ns=1;i=5043 | Aktueller Winkel Gelenk 3 (Elbow) |
| `ro_arm_joint4_angle_centi_deg` | Input (R) | 255 | 1 | int16 | 0.01deg |  |  | decimal |  | ns=1;i=5044 | Aktueller Winkel Gelenk 4 (Wrist) |
| `ro_arm_joint5_angle_centi_deg` | Input (R) | 256 | 1 | int16 | 0.01deg |  |  | decimal |  | ns=1;i=5045 | Aktueller Winkel Gelenk 5 (Roll) |
| `ro_arm_joint6_angle_centi_deg` | Input (R) | 257 | 1 | int16 | 0.01deg |  |  | decimal |  | ns=1;i=5046 | Aktueller Winkel Gelenk 6 (Gripper) |

### Waegezelle

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `scale_a_weight` | Input (R) | 220..221 | 2 | int32[^byteorder] |  |  |  | decimal |  | ns=1;i=5047 | Gewicht Waegezelle A |
| `scale_a_status` | Input (R) | 222 | 1 | uint16 |  |  |  | status |  | ns=1;i=5048 | 0 = ok, ungleich 0 = Fehler (aktuell: 1 = keine gueltige Messung) |
| `scale_b_raw` | Input (R) | 223..224 | 2 | uint32[^byteorder] |  |  |  | decimal |  | ns=1;i=5049 | Rohwert Waegezelle B (Reserve/Erweiterung) |
| `scale_b_status` | Input (R) | 225 | 1 | uint16 |  |  |  | status |  | ns=1;i=5050 | 0 = ok, ungleich 0 = Fehler -- auf dieser Platinen-Revision nicht bestueckt, Register bleibt ungeschrieben (zeigt daher immer 0) |

### Stepper-Steuerung

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `stepper_enable` | Holding (R/W) | 60 | 1 | uint16 |  |  |  | bool | PB7 | ns=1;i=5051 | Stepper-Treiber global freigeben |
| `stepper1_target` | Holding (R/W) | 61..62 | 2 | int32[^byteorder] |  |  |  | decimal |  | ns=1;i=5052 | Zielposition Stepper 1 |
| `stepper1_speed` | Holding (R/W) | 63 | 1 | uint16 | Schritte/s |  |  | decimal |  | ns=1;i=5053 | Geschwindigkeit Stepper 1 |
| `stepper1_accel` | Holding (R/W) | 64 | 1 | uint16 | Schritte/s^2 |  |  | decimal |  | ns=1;i=5054 | Beschleunigung Stepper 1 |
| `stepper2_target` | Holding (R/W) | 65..66 | 2 | int32[^byteorder] |  |  |  | decimal |  | ns=1;i=5055 | Zielposition Stepper 2 |
| `stepper2_speed` | Holding (R/W) | 67 | 1 | uint16 | Schritte/s |  |  | decimal |  | ns=1;i=5056 | Geschwindigkeit Stepper 2 |
| `stepper2_accel` | Holding (R/W) | 68 | 1 | uint16 | Schritte/s^2 |  |  | decimal |  | ns=1;i=5057 | Beschleunigung Stepper 2 |
| `stepper1_start_homing` | Holding (R/W) | 69 | 1 | uint16 |  | 0 | 255 | decimal |  | ns=1;i=5058 | 1=Homing starten (stoppt Stepper1, sensorloses Homing per StallGuard; beim Start automatisch, wenn die Motorversorgung da ist), 0=erfolgreich beendet, 2=Timeout (kein Anschlag), 3=TMC2209 antwortet nicht |
| `stepper2_start_homing` | Holding (R/W) | 70 | 1 | uint16 |  | 0 | 255 | decimal |  | ns=1;i=5059 | 1=Homing starten (stoppt Stepper2, sensorloses Homing per StallGuard; beim Start automatisch, wenn die Motorversorgung da ist), 0=erfolgreich beendet, 2=Timeout (kein Anschlag), 3=TMC2209 antwortet nicht |

### Stepper-Status

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `stepper1_position` | Input (R) | 240..241 | 2 | int32[^byteorder] |  |  |  | decimal |  | ns=1;i=5060 | Aktuelle Position Stepper 1 |
| `stepper1_status` | Input (R) | 242 | 1 | uint16 |  |  |  | binary |  | ns=1;i=5061 | Bit0 Enabled, Bit1 Moving, Bit2 Homing fehlgeschlagen, Bit3 InPosition, Bit4 Homing laeuft, Bit5 referenziert (Homing erfolgreich, beim Start oder per StartHoming) |
| `stepper2_position` | Input (R) | 243..244 | 2 | int32[^byteorder] |  |  |  | decimal |  | ns=1;i=5062 | Aktuelle Position Stepper 2 |
| `stepper2_status` | Input (R) | 245 | 1 | uint16 |  |  |  | binary |  | ns=1;i=5063 | Bit0 Enabled, Bit1 Moving, Bit2 Homing fehlgeschlagen, Bit3 InPosition, Bit4 Homing laeuft, Bit5 referenziert (Homing erfolgreich, beim Start oder per StartHoming) |

### ToF-Sensoren

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `tof1_distance_mm` | Input (R) | 140 | 1 | uint16 | mm |  |  | decimal | I2C_1 (IRQ Rev1 PC15, Rev2 PC14) | ns=1;i=5064 | Abstand Sensor 1 (VL53L0X) |
| `tof1_status` | Input (R) | 141 | 1 | uint16 |  |  |  | status | I2C_1 (IRQ Rev1 PC15, Rev2 PC14) | ns=1;i=5065 | 0 = ok, ungleich 0 = Fehler (aktuell: 1 = nicht erkannt/keine gueltige Messung) |
| `tof2_distance_mm` | Input (R) | 142 | 1 | uint16 | mm |  |  | decimal | I2C_2 (IRQ Rev1 PH1, Rev2 PE15) | ns=1;i=5066 | Abstand Sensor 2 (VL53L0X) |
| `tof2_status` | Input (R) | 143 | 1 | uint16 |  |  |  | status | I2C_2 (IRQ Rev1 PH1, Rev2 PE15) | ns=1;i=5067 | 0 = ok, ungleich 0 = Fehler (aktuell: 1 = nicht erkannt/keine gueltige Messung) |
| `tof3_distance_mm` | Input (R) | 144 | 1 | uint16 | mm |  |  | decimal | I2C_4 (IRQ Rev1 PA3, Rev2 PD10) | ns=1;i=5068 | Abstand Sensor 3 (VL53L0X) |
| `tof3_status` | Input (R) | 145 | 1 | uint16 |  |  |  | status | I2C_4 (IRQ Rev1 PA3, Rev2 PD10) | ns=1;i=5069 | 0 = ok, ungleich 0 = Fehler (aktuell: 1 = nicht erkannt/keine gueltige Messung) |

### Pneumatikventile

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `valve1` | Holding (R/W) | 20 | 1 | uint16 |  |  |  | bool | Rev1 PD10, Rev2 PA8 | ns=1;i=5070 | Pneumatikventil 1 |
| `valve2` | Holding (R/W) | 21 | 1 | uint16 |  |  |  | bool | Rev1 PD5, Rev2 PC6 | ns=1;i=5071 | Pneumatikventil 2 |
| `valve3` | Holding (R/W) | 22 | 1 | uint16 |  |  |  | bool | PD11 | ns=1;i=5072 | Pneumatikventil 3 |
| `vacuum_valve` | Holding (R/W) | 23 | 1 | uint16 |  |  |  | bool | Rev2 PB14 | ns=1;i=5073 | Vakuumventil (nur Hardware-Revision 2) |
| `vacuum_pump` | Holding (R/W) | 24 | 1 | uint16 |  |  |  | bool | Rev2 PC11 | ns=1;i=5074 | Vakuumpumpe (nur Hardware-Revision 2) |

### WS2812

| Bezeichnung | Bank | Adresse | Worte | Typ | Einheit | Min | Max | Anzeige | GPIO/I2C | OPC-UA-NodeId | Beschreibung |
|---|---|---:|---:|---|---|---:|---:|---|---|---|---|
| `ws2812_ch1_pattern` | Holding (R/W) | 100 | 1 | uint16 |  |  |  | decimal | PE5/TIM15_CH1 | ns=1;i=5075 | Index in fest im Code hinterlegte Farbmuster-Tabelle |
| `ws2812_ch2_pattern` | Holding (R/W) | 101 | 1 | uint16 |  |  |  | decimal | PE6/TIM15_CH2 | ns=1;i=5076 | Index in fest im Code hinterlegte Farbmuster-Tabelle |

## Input-Register nach Adresse (FC04)

| Adresse | Worte | Gruppe | Bezeichnung | Typ | Einheit |
|---:|---:|---|---|---|---|
| 0 | 1 | Diagnostik | `health_state` | Bitfeld (HealthBits) |  |
| 1..6 | 6 | Diagnostik | `chip_id` | Struct (Uid96)[^byteorder] |  |
| 7 | 1 | Diagnostik | `fw_version_major` | uint16 |  |
| 8 | 1 | Diagnostik | `fw_version_minor` | uint16 |  |
| 9 | 1 | Diagnostik | `fw_version_patch` | uint16 |  |
| 10 | 1 | Diagnostik | `timer_tick` | uint16 | ticks |
| 11 | 1 | Diagnostik | `free_heap_kib` | uint16 | KiB |
| 12 | 1 | Diagnostik | `cpu_temperature_c` | int16 | C |
| 50..51 | 2 | CAN-Bus | `can_tx_count` | uint32[^byteorder] |  |
| 52..53 | 2 | CAN-Bus | `can_rx_count` | uint32[^byteorder] |  |
| 54..55 | 2 | CAN-Bus | `can_error_count` | uint32[^byteorder] |  |
| 56 | 1 | CAN-Bus | `can_last_error` | uint16 |  |
| 57 | 1 | CAN-Bus | `can_bus_state` | uint16 |  |
| 80 | 1 | Ethernet | `eth_link_status` | uint16 |  |
| 81 | 1 | Ethernet | `eth_link_speed` | uint16 | Mbit/s |
| 82 | 1 | Ethernet | `eth_link_duplex` | uint16 |  |
| 83..84 | 2 | Ethernet | `eth_tx_count` | uint32[^byteorder] |  |
| 85..86 | 2 | Ethernet | `eth_rx_count` | uint32[^byteorder] |  |
| 87..88 | 2 | Ethernet | `eth_error_count` | uint32[^byteorder] |  |
| 89 | 1 | Ethernet | `eth_last_error` | uint16 |  |
| 110 | 1 | Stromversorgung | `pwr_bus_voltage_mv` | uint16 | mV |
| 111 | 1 | Stromversorgung | `pwr_shunt_voltage_uv` | int16 | uV |
| 112 | 1 | Stromversorgung | `pwr_current_ma` | int16 | mA |
| 113 | 1 | Stromversorgung | `pwr_power_mw` | uint16 | mW |
| 114 | 1 | Stromversorgung | `pwr_pd_voltage_mv` | uint16 | mV |
| 115 | 1 | Stromversorgung | `pwr_pd_current_ma` | uint16 | mA |
| 116 | 1 | Stromversorgung | `pwr_pd_status` | uint16 |  |
| 117 | 1 | Stromversorgung | `pwr_status` | uint16 |  |
| 140 | 1 | ToF-Sensoren | `tof1_distance_mm` | uint16 | mm |
| 141 | 1 | ToF-Sensoren | `tof1_status` | uint16 |  |
| 142 | 1 | ToF-Sensoren | `tof2_distance_mm` | uint16 | mm |
| 143 | 1 | ToF-Sensoren | `tof2_status` | uint16 |  |
| 144 | 1 | ToF-Sensoren | `tof3_distance_mm` | uint16 | mm |
| 145 | 1 | ToF-Sensoren | `tof3_status` | uint16 |  |
| 160 | 1 | Farbsensor | `color_clear` | uint16 |  |
| 161 | 1 | Farbsensor | `color_red` | uint16 |  |
| 162 | 1 | Farbsensor | `color_green` | uint16 |  |
| 163 | 1 | Farbsensor | `color_blue` | uint16 |  |
| 180 | 1 | Lichtschranken | `lightbarrier1` | uint16 |  |
| 181 | 1 | Lichtschranken | `lightbarrier2` | uint16 |  |
| 182 | 1 | Lichtschranken | `lightbarrier3` | uint16 |  |
| 200 | 1 | Drucksensor | `pressure_raw` | uint16 |  |
| 220..221 | 2 | Waegezelle | `scale_a_weight` | int32[^byteorder] |  |
| 222 | 1 | Waegezelle | `scale_a_status` | uint16 |  |
| 223..224 | 2 | Waegezelle | `scale_b_raw` | uint32[^byteorder] |  |
| 225 | 1 | Waegezelle | `scale_b_status` | uint16 |  |
| 240..241 | 2 | Stepper-Status | `stepper1_position` | int32[^byteorder] |  |
| 242 | 1 | Stepper-Status | `stepper1_status` | uint16 |  |
| 243..244 | 2 | Stepper-Status | `stepper2_position` | int32[^byteorder] |  |
| 245 | 1 | Stepper-Status | `stepper2_status` | uint16 |  |
| 250 | 1 | RoArm-M3 Status | `status` | Bitfeld (RoArmStatusBits) |  |
| 251 | 1 | RoArm-M3 Status | `ro_arm_active_mission` | int16 |  |
| 252 | 1 | RoArm-M3 Status | `ro_arm_joint1_angle_centi_deg` | int16 | 0.01deg |
| 253 | 1 | RoArm-M3 Status | `ro_arm_joint2_angle_centi_deg` | int16 | 0.01deg |
| 254 | 1 | RoArm-M3 Status | `ro_arm_joint3_angle_centi_deg` | int16 | 0.01deg |
| 255 | 1 | RoArm-M3 Status | `ro_arm_joint4_angle_centi_deg` | int16 | 0.01deg |
| 256 | 1 | RoArm-M3 Status | `ro_arm_joint5_angle_centi_deg` | int16 | 0.01deg |
| 257 | 1 | RoArm-M3 Status | `ro_arm_joint6_angle_centi_deg` | int16 | 0.01deg |

## Holding-Register nach Adresse (FC03/06/16)

| Adresse | Worte | Gruppe | Bezeichnung | Typ | Einheit |
|---:|---:|---|---|---|---|
| 20 | 1 | Pneumatikventile | `valve1` | uint16 |  |
| 21 | 1 | Pneumatikventile | `valve2` | uint16 |  |
| 22 | 1 | Pneumatikventile | `valve3` | uint16 |  |
| 23 | 1 | Pneumatikventile | `vacuum_valve` | uint16 |  |
| 24 | 1 | Pneumatikventile | `vacuum_pump` | uint16 |  |
| 40 | 1 | Motor-PWM | `conveyor_pwm` | uint16 | Promille |
| 41 | 1 | Motor-PWM | `compressor_pwm` | uint16 | Promille |
| 60 | 1 | Stepper-Steuerung | `stepper_enable` | uint16 |  |
| 61..62 | 2 | Stepper-Steuerung | `stepper1_target` | int32[^byteorder] |  |
| 63 | 1 | Stepper-Steuerung | `stepper1_speed` | uint16 | Schritte/s |
| 64 | 1 | Stepper-Steuerung | `stepper1_accel` | uint16 | Schritte/s^2 |
| 65..66 | 2 | Stepper-Steuerung | `stepper2_target` | int32[^byteorder] |  |
| 67 | 1 | Stepper-Steuerung | `stepper2_speed` | uint16 | Schritte/s |
| 68 | 1 | Stepper-Steuerung | `stepper2_accel` | uint16 | Schritte/s^2 |
| 69 | 1 | Stepper-Steuerung | `stepper1_start_homing` | uint16 |  |
| 70 | 1 | Stepper-Steuerung | `stepper2_start_homing` | uint16 |  |
| 100 | 1 | WS2812 | `ws2812_ch1_pattern` | uint16 |  |
| 101 | 1 | WS2812 | `ws2812_ch2_pattern` | uint16 |  |
| 110 | 1 | RoArm-M3 Steuerung | `ro_arm_mission_control` | int16 |  |

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

    Beispiel: `scale_a_weight` (int32, Input-Register 220..221) mit dem Wert -100000 = 0xFFFE7960 (Zweierkomplement):

    - Register 220 (High-Word) = 0xFFFE, Register 221 (Low-Word) = 0x7960
    - Datenbytes in der FC04-Antwort (ab Startadresse 220, Anzahl 2): `FF FE 79 60`
    - Zusammensetzen: `(0xFFFE << 16) | 0x7960` = 0xFFFE7960, als int32 interpretiert -100000
    - Bei Clients mit einstellbarer Wort-Reihenfolge (z. B. "Word Swap"/"CDAB") also KEINEN Wort-Tausch waehlen.
