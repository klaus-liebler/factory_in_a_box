// GENERIERT von tools/generate-register-map.mjs aus register-map.json -- nicht von Hand
// editieren. Aenderungen an der Register-Map gehoeren in register-map.json, anschliessend
// "node tools/generate-register-map.mjs" erneut ausfuehren.
//
// Nur per #include aus Core/Src/modbus_register_model.hh eingebunden (innerhalb des dort
// bereits geoeffneten namespace-Blocks), nicht eigenstaendig uebersetzbar -- enthaelt bewusst
// nur constexpr-Zeilen, keine eigene namespace-Deklaration/Klammern.
//
// Adressen sind Offsets in ModbusRegisterModel::input_registers_.
// 32-Bit-Werte belegen zwei Register (High-Word zuerst), siehe MbapHeader::serialize.
// Block-Markierungen heissen *_REGION_START/*_REGION_END statt *_BASE/*_END, da z.B.
// ETH_BASE/PWR_BASE bereits als CMSIS-Peripherie-Basisadressen vergeben sind.

// Diagnostik (Region-Start 0, Reserve bis 49)
constexpr uint16_t HEALTH_STATE = 0; // Health-Bitfeld (siehe healthBits)
constexpr uint16_t CHIP_ID_W0_HI = 1; // Chip-UID Wort 0, High
constexpr uint16_t CHIP_ID_W0_LO = 2; // Chip-UID Wort 0, Low
constexpr uint16_t CHIP_ID_W1_HI = 3; // Chip-UID Wort 1, High
constexpr uint16_t CHIP_ID_W1_LO = 4; // Chip-UID Wort 1, Low
constexpr uint16_t CHIP_ID_W2_HI = 5; // Chip-UID Wort 2, High
constexpr uint16_t CHIP_ID_W2_LO = 6; // Chip-UID Wort 2, Low
constexpr uint16_t FW_VERSION_MAJOR = 7; // Firmware-Version Major
constexpr uint16_t FW_VERSION_MINOR = 8; // Firmware-Version Minor
constexpr uint16_t FW_VERSION_PATCH = 9; // Firmware-Version Patch
constexpr uint16_t TIMER_TICK = 10; // Freilaufender Systick-Zaehler, Ueberlauf nach 65535 -- [ticks]
constexpr uint16_t DIAGNOSTICS_REGION_START = 0;
constexpr uint16_t DIAGNOSTICS_REGION_END   = 49;

// CAN-Bus (Region-Start 50, Reserve bis 79)
constexpr uint16_t CAN_TX_COUNT_HI = 50; // Gesendete CAN-Frames, High
constexpr uint16_t CAN_TX_COUNT_LO = 51; // Gesendete CAN-Frames, Low
constexpr uint16_t CAN_RX_COUNT_HI = 52; // Empfangene CAN-Frames, High
constexpr uint16_t CAN_RX_COUNT_LO = 53; // Empfangene CAN-Frames, Low
constexpr uint16_t CAN_ERROR_COUNT_HI = 54; // CAN-Fehlerzaehler, High
constexpr uint16_t CAN_ERROR_COUNT_LO = 55; // CAN-Fehlerzaehler, Low
constexpr uint16_t CAN_LAST_ERROR = 56; // Letzter CAN-Fehlercode
constexpr uint16_t CAN_BUS_STATE = 57; // CAN-Bus-Zustand
constexpr uint16_t CAN_REGION_START = 50;
constexpr uint16_t CAN_REGION_END   = 79;

// Ethernet (Region-Start 80, Reserve bis 109)
constexpr uint16_t ETH_LINK_STATUS = 80; // 0 = down, 1 = up
constexpr uint16_t ETH_LINK_SPEED = 81; // 0=10M, 1=100M, 2=1000M
constexpr uint16_t ETH_LINK_DUPLEX = 82; // 0=half, 1=full
constexpr uint16_t ETH_TX_COUNT_HI = 83; // Gesendete Ethernet-Frames, High
constexpr uint16_t ETH_TX_COUNT_LO = 84; // Gesendete Ethernet-Frames, Low
constexpr uint16_t ETH_RX_COUNT_HI = 85; // Empfangene Ethernet-Frames, High
constexpr uint16_t ETH_RX_COUNT_LO = 86; // Empfangene Ethernet-Frames, Low
constexpr uint16_t ETH_ERROR_COUNT_HI = 87; // Ethernet-Fehlerzaehler, High
constexpr uint16_t ETH_ERROR_COUNT_LO = 88; // Ethernet-Fehlerzaehler, Low
constexpr uint16_t ETH_LAST_ERROR = 89; // Letzter Ethernet/PHY-Fehlercode
constexpr uint16_t ETH_REGION_START = 80;
constexpr uint16_t ETH_REGION_END   = 109;

// Stromversorgung (Region-Start 110, Reserve bis 139)
constexpr uint16_t PWR_BUS_VOLTAGE_MV = 110; // Busspannung (INA226) -- [mV]
constexpr uint16_t PWR_SHUNT_VOLTAGE_UV = 111; // Shunt-Spannung (INA226) -- [uV] -- signed
constexpr uint16_t PWR_CURRENT_MA = 112; // Stromaufnahme (INA226) -- [mA] -- signed
constexpr uint16_t PWR_POWER_MW = 113; // Leistungsaufnahme (INA226) -- [mW]
constexpr uint16_t PWR_PD_VOLTAGE_MV = 114; // Aktiv ausgehandelte USB-PD-Spannung, PDSink::activeVoltage -- [mV]
constexpr uint16_t PWR_PD_CURRENT_MA = 115; // Aktiv ausgehandelter USB-PD-Maximalstrom, PDSink::activeCurrent -- [mA]
constexpr uint16_t PWR_PD_STATUS = 116; // 0 = keine Quelle verbunden, 1 = verbunden/Kontrakt aktiv
constexpr uint16_t PWR_REGION_START = 110;
constexpr uint16_t PWR_REGION_END   = 139;

// ToF-Sensoren (Region-Start 140, Reserve bis 159)
constexpr uint16_t TOF1_DISTANCE_MM = 140; // Abstand Sensor 1 (VL53L0X) -- [mm] -- I2C_1, IRQ PC15
constexpr uint16_t TOF1_STATUS = 141; // Status Sensor 1 -- I2C_1, IRQ PC15
constexpr uint16_t TOF2_DISTANCE_MM = 142; // Abstand Sensor 2 (VL53L0X) -- [mm] -- I2C_2, IRQ PH1
constexpr uint16_t TOF2_STATUS = 143; // Status Sensor 2 -- I2C_2, IRQ PH1
constexpr uint16_t TOF3_DISTANCE_MM = 144; // Abstand Sensor 3 (VL53L0X) -- [mm] -- I2C_4, IRQ PA3
constexpr uint16_t TOF3_STATUS = 145; // Status Sensor 3 -- I2C_4, IRQ PA3
constexpr uint16_t TOF_REGION_START = 140;
constexpr uint16_t TOF_REGION_END   = 159;

// Farbsensor (Region-Start 160, Reserve bis 179)
constexpr uint16_t COLOR_CLEAR = 160; // Farbsensor TCS34725, Clear-Kanal -- I2C_1, IRQ PH0
constexpr uint16_t COLOR_RED = 161; // Farbsensor TCS34725, Rot-Kanal -- I2C_1, IRQ PH0
constexpr uint16_t COLOR_GREEN = 162; // Farbsensor TCS34725, Gruen-Kanal -- I2C_1, IRQ PH0
constexpr uint16_t COLOR_BLUE = 163; // Farbsensor TCS34725, Blau-Kanal -- I2C_1, IRQ PH0
constexpr uint16_t COLOR_REGION_START = 160;
constexpr uint16_t COLOR_REGION_END   = 179;

// Lichtschranken (Region-Start 180, Reserve bis 199)
constexpr uint16_t LIGHTBARRIER1 = 180; // Lichtschranke 1 (NPN, aktiv=LOW) -- PC6
constexpr uint16_t LIGHTBARRIER2 = 181; // Lichtschranke 2 (NPN, aktiv=LOW) -- PC7
constexpr uint16_t LIGHTBARRIER3 = 182; // Lichtschranke 3 (NPN, aktiv=LOW) -- nur auf realem Board verdrahtet -- PC8
constexpr uint16_t LIGHTBARRIER_REGION_START = 180;
constexpr uint16_t LIGHTBARRIER_REGION_END   = 199;

// Drucksensor (Region-Start 200, Reserve bis 219)
constexpr uint16_t PRESSURE_RAW = 200; // Analoger Drucksensor, Rohwert (ADC1 CH10) -- PA4
constexpr uint16_t PRESSURE_REGION_START = 200;
constexpr uint16_t PRESSURE_REGION_END   = 219;

// Waegezelle (Region-Start 220, Reserve bis 239)
constexpr uint16_t SCALE_A_WEIGHT_HI = 220; // Gewicht Waegezelle A, High -- signed
constexpr uint16_t SCALE_A_WEIGHT_LO = 221; // Gewicht Waegezelle A, Low
constexpr uint16_t SCALE_A_STATUS = 222; // Status Waegezelle A
constexpr uint16_t SCALE_B_RAW_HI = 223; // Rohwert Waegezelle B, High (Reserve/Erweiterung)
constexpr uint16_t SCALE_B_RAW_LO = 224; // Rohwert Waegezelle B, Low
constexpr uint16_t SCALE_B_STATUS = 225; // Status Waegezelle B
constexpr uint16_t SCALE_REGION_START = 220;
constexpr uint16_t SCALE_REGION_END   = 239;

// Stepper-Status (Region-Start 240, Reserve bis 269)
constexpr uint16_t STEPPER1_POSITION_HI = 240; // Aktuelle Position Stepper 1, High -- signed
constexpr uint16_t STEPPER1_POSITION_LO = 241; // Aktuelle Position Stepper 1, Low
constexpr uint16_t STEPPER1_STATUS = 242; // Bit0 Enabled, Bit1 Moving, Bit2 Error/StallGuard, Bit3 InPosition
constexpr uint16_t STEPPER2_POSITION_HI = 243; // Aktuelle Position Stepper 2, High -- signed
constexpr uint16_t STEPPER2_POSITION_LO = 244; // Aktuelle Position Stepper 2, Low
constexpr uint16_t STEPPER2_STATUS = 245; // Bit0 Enabled, Bit1 Moving, Bit2 Error/StallGuard, Bit3 InPosition
constexpr uint16_t STEPPER_STATUS_REGION_START = 240;
constexpr uint16_t STEPPER_STATUS_REGION_END   = 269;

// HealthState-Bitfeld (Input::HEALTH_STATE), s. register-map.json healthBits
constexpr uint16_t HEALTH_BIT_OK = 0;
constexpr uint16_t HEALTH_BIT_OVERTEMPERATURE = 1;
constexpr uint16_t HEALTH_BIT_POWER_FAULT = 2;
constexpr uint16_t HEALTH_BIT_CAN_ERROR = 3;
constexpr uint16_t HEALTH_BIT_ETH_LINK_DOWN = 4;
constexpr uint16_t HEALTH_BIT_I2C_SENSOR_FAULT = 5;
constexpr uint16_t HEALTH_BIT_STEPPER_FAULT = 6;
