// GENERIERT von tools/generate-register-map.mjs aus register-map.json -- nicht von Hand
// editieren. Aenderungen an der Register-Map gehoeren in register-map.json, anschliessend
// "node tools/generate-register-map.mjs" erneut ausfuehren.

export type RegisterBank = "input" | "holding";
// Eine Darstellungsform fuers Lesen UND Schreiben (s. register-panel.ts):
//   - "decimal" (Default): Zahl; bei Holding-Registern generisches Zahlenfeld + Schreiben-Button.
//   - "hex" / "binary": Zahl als 0x.../0b...-String; Schreiben wie "decimal".
//   - "bool": bei Input-Registern grau/gruen eingefaerbtes Badge (0=aus/1=an), bei
//     Holding-Registern zusaetzlich ein Toggle-Switch zum Schreiben.
//   - "bool-error": wie "bool", aber grau/rot statt grau/gruen -- fuer Register, bei denen 1
//     einen Fehler-/Alarmzustand meldet statt eines neutralen "an" (z.B. PWR_ALERT).
//   - "status": Fehlercode-Konvention (0 = ok, ungleich 0 = Fehler) -- 0 grau/neutral, jeder
//     andere Wert rot MIT der tatsaechlichen Zahl (nicht nur "1"), fuer *_STATUS-Register, die
//     kuenftig auch verschiedene Fehlercodes tragen koennten (aktuell ueberall nur 0/1).
//   - "range": Slider (braucht min/max); nur fuer Holding-Register sinnvoll.
export type RegisterDisplay = "decimal" | "hex" | "binary" | "bool" | "bool-error" | "status" | "range";

export interface RegisterDef {
	name: string;
	address: number;
	bank: RegisterBank;
	display: RegisterDisplay;
	description?: string;
	unit?: string;
	signed?: boolean;
	gpio?: string;
	i2c?: { bus: string; irqPin?: string };
	min?: number;
	max?: number;
	// Fasst "count" aufeinanderfolgende Register (ab "address", MSB zuerst) zu einem
	// gemeinsamen Anzeigewert zusammen (z.B. 6 Register -> eine 96-Bit Chip-ID als Hex-String,
	// oder 2 Register -> ein 32-Bit-Zaehlerstand). Nur lesend, nur sinnvoll bei Input-Registern.
	combine?: { count: number };
}

export interface RegisterRegion {
	title: string;
	registers: RegisterDef[];
}

export const REGIONS: RegisterRegion[] = [
	{
		title: "Diagnostik",
		registers: [
			{ name: "HEALTH_STATE", address: 0, bank: "input", description: "Health-Bitfeld (siehe healthBits)", display: "binary" },
			{ name: "CHIP_ID", address: 1, bank: "input", description: "96-Bit Chip-UID", display: "hex", combine: { count: 6 } },
			{ name: "FW_VERSION_MAJOR", address: 7, bank: "input", description: "Firmware-Version Major", display: "decimal" },
			{ name: "FW_VERSION_MINOR", address: 8, bank: "input", description: "Firmware-Version Minor", display: "decimal" },
			{ name: "FW_VERSION_PATCH", address: 9, bank: "input", description: "Firmware-Version Patch", display: "decimal" },
			{ name: "TIMER_TICK", address: 10, bank: "input", description: "Freilaufender Systick-Zaehler, Ueberlauf nach 65535", unit: "ticks", display: "decimal" },
			{ name: "FREE_HEAP_KIB", address: 11, bank: "input", description: "Freier newlib-Heap (High-Water-Mark bis zur MSP-Stack-Reserve, s. sysmem.c) -- freigegebene, aber noch sbrk'te Bloecke zaehlen nicht mit, also eine pessimistische untere Schranke", unit: "KiB", display: "decimal" },
			{ name: "CPU_TEMPERATURE_C", address: 12, bank: "input", description: "Chip-Temperatur (interner DTS-Sensor, RM0481 Kap. 29, werksseitig je Chip kalibriert)", unit: "C", signed: true, display: "decimal" }
		]
	},
	{
		title: "CAN-Bus",
		registers: [
			{ name: "CAN_TX_COUNT", address: 50, bank: "input", description: "Gesendete CAN-Frames", display: "decimal", combine: { count: 2 } },
			{ name: "CAN_RX_COUNT", address: 52, bank: "input", description: "Empfangene CAN-Frames", display: "decimal", combine: { count: 2 } },
			{ name: "CAN_ERROR_COUNT", address: 54, bank: "input", description: "CAN-Fehlerzaehler", display: "decimal", combine: { count: 2 } },
			{ name: "CAN_LAST_ERROR", address: 56, bank: "input", description: "Letzter CAN-Fehlercode", display: "decimal" },
			{ name: "CAN_BUS_STATE", address: 57, bank: "input", description: "CAN-Bus-Zustand", display: "decimal" }
		]
	},
	{
		title: "Ethernet",
		registers: [
			{ name: "ETH_LINK_STATUS", address: 80, bank: "input", description: "0 = down, 1 = up", display: "bool" },
			{ name: "ETH_LINK_SPEED", address: 81, bank: "input", description: "Verbindungsgeschwindigkeit (0 = kein Link)", unit: "Mbit/s", display: "decimal" },
			{ name: "ETH_LINK_DUPLEX", address: 82, bank: "input", description: "0=half, 1=full", display: "bool" },
			{ name: "ETH_TX_COUNT", address: 83, bank: "input", description: "Gesendete Ethernet-Frames", display: "decimal", combine: { count: 2 } },
			{ name: "ETH_RX_COUNT", address: 85, bank: "input", description: "Empfangene Ethernet-Frames", display: "decimal", combine: { count: 2 } },
			{ name: "ETH_ERROR_COUNT", address: 87, bank: "input", description: "Ethernet-Fehlerzaehler", display: "decimal", combine: { count: 2 } },
			{ name: "ETH_LAST_ERROR", address: 89, bank: "input", description: "Letzter Ethernet/PHY-Fehlercode", display: "decimal" }
		]
	},
	{
		title: "Stromversorgung",
		registers: [
			{ name: "PWR_BUS_VOLTAGE_MV", address: 110, bank: "input", description: "Busspannung (INA226, 2 mOhm Shunt)", unit: "mV", i2c: { bus: "I2C_4" }, display: "decimal" },
			{ name: "PWR_SHUNT_VOLTAGE_UV", address: 111, bank: "input", description: "Shunt-Spannung (INA226, 2 mOhm Shunt)", unit: "uV", signed: true, i2c: { bus: "I2C_4" }, display: "decimal" },
			{ name: "PWR_CURRENT_MA", address: 112, bank: "input", description: "Stromaufnahme (INA226, 2 mOhm Shunt)", unit: "mA", signed: true, i2c: { bus: "I2C_4" }, display: "decimal" },
			{ name: "PWR_POWER_MW", address: 113, bank: "input", description: "Leistungsaufnahme (INA226)", unit: "mW", i2c: { bus: "I2C_4" }, display: "decimal" },
			{ name: "PWR_PD_VOLTAGE_MV", address: 114, bank: "input", description: "Aktiv ausgehandelte USB-PD-Spannung, PDSink::activeVoltage", unit: "mV", display: "decimal" },
			{ name: "PWR_PD_CURRENT_MA", address: 115, bank: "input", description: "Aktiv ausgehandelter USB-PD-Maximalstrom, PDSink::activeCurrent", unit: "mA", display: "decimal" },
			{ name: "PWR_PD_STATUS", address: 116, bank: "input", description: "0 = verbunden und Spannung eingestellt, 1 = verbunden aber Spannung nicht einstellbar, 2 = kein USB-PD-Netzteil erkannt", display: "status" },
			{ name: "PWR_STATUS", address: 117, bank: "input", description: "0 = ok, ungleich 0 = Fehler (aktuell: 1 = INA226 nicht erkannt/Init fehlgeschlagen)", i2c: { bus: "I2C_4" }, display: "status" }
		]
	},
	{
		title: "ToF-Sensoren",
		registers: [
			{ name: "TOF1_DISTANCE_MM", address: 140, bank: "input", description: "Abstand Sensor 1 (VL53L0X)", unit: "mm", i2c: { bus: "I2C_1", irqPin: "PC15" }, display: "decimal" },
			{ name: "TOF1_STATUS", address: 141, bank: "input", description: "0 = ok, ungleich 0 = Fehler (aktuell: 1 = nicht erkannt/keine gueltige Messung)", i2c: { bus: "I2C_1", irqPin: "PC15" }, display: "status" },
			{ name: "TOF2_DISTANCE_MM", address: 142, bank: "input", description: "Abstand Sensor 2 (VL53L0X)", unit: "mm", i2c: { bus: "I2C_2", irqPin: "PH1" }, display: "decimal" },
			{ name: "TOF2_STATUS", address: 143, bank: "input", description: "0 = ok, ungleich 0 = Fehler (aktuell: 1 = nicht erkannt/keine gueltige Messung)", i2c: { bus: "I2C_2", irqPin: "PH1" }, display: "status" },
			{ name: "TOF3_DISTANCE_MM", address: 144, bank: "input", description: "Abstand Sensor 3 (VL53L0X)", unit: "mm", i2c: { bus: "I2C_4", irqPin: "PA3" }, display: "decimal" },
			{ name: "TOF3_STATUS", address: 145, bank: "input", description: "0 = ok, ungleich 0 = Fehler (aktuell: 1 = nicht erkannt/keine gueltige Messung)", i2c: { bus: "I2C_4", irqPin: "PA3" }, display: "status" }
		]
	},
	{
		title: "Farbsensor",
		registers: [
			{ name: "COLOR_CLEAR", address: 160, bank: "input", description: "Farbsensor TCS34725, Clear-Kanal", i2c: { bus: "I2C_1", irqPin: "PH0" }, display: "decimal" },
			{ name: "COLOR_RED", address: 161, bank: "input", description: "Farbsensor TCS34725, Rot-Kanal", i2c: { bus: "I2C_1", irqPin: "PH0" }, display: "decimal" },
			{ name: "COLOR_GREEN", address: 162, bank: "input", description: "Farbsensor TCS34725, Gruen-Kanal", i2c: { bus: "I2C_1", irqPin: "PH0" }, display: "decimal" },
			{ name: "COLOR_BLUE", address: 163, bank: "input", description: "Farbsensor TCS34725, Blau-Kanal", i2c: { bus: "I2C_1", irqPin: "PH0" }, display: "decimal" }
		]
	},
	{
		title: "Lichtschranken",
		registers: [
			{ name: "LIGHTBARRIER1", address: 180, bank: "input", description: "Lichtschranke 1 (NPN, aktiv=LOW)", gpio: "PC6", display: "bool" },
			{ name: "LIGHTBARRIER2", address: 181, bank: "input", description: "Lichtschranke 2 (NPN, aktiv=LOW)", gpio: "PC7", display: "bool" },
			{ name: "LIGHTBARRIER3", address: 182, bank: "input", description: "Lichtschranke 3 (NPN, aktiv=LOW) -- nur auf realem Board verdrahtet", gpio: "PC8", display: "bool" }
		]
	},
	{
		title: "Drucksensor",
		registers: [
			{ name: "PRESSURE_RAW", address: 200, bank: "input", description: "Analoger Drucksensor, Rohwert (ADC1 CH10)", gpio: "PA4", display: "decimal" }
		]
	},
	{
		title: "Waegezelle",
		registers: [
			{ name: "SCALE_A_WEIGHT", address: 220, bank: "input", description: "Gewicht Waegezelle A", signed: true, display: "decimal", combine: { count: 2 } },
			{ name: "SCALE_A_STATUS", address: 222, bank: "input", description: "0 = ok, ungleich 0 = Fehler (aktuell: 1 = keine gueltige Messung)", display: "status" },
			{ name: "SCALE_B_RAW", address: 223, bank: "input", description: "Rohwert Waegezelle B (Reserve/Erweiterung)", display: "decimal", combine: { count: 2 } },
			{ name: "SCALE_B_STATUS", address: 225, bank: "input", description: "0 = ok, ungleich 0 = Fehler -- auf dieser Platinen-Revision nicht bestueckt, Register bleibt ungeschrieben (zeigt daher immer 0)", display: "status" }
		]
	},
	{
		title: "Stepper-Status",
		registers: [
			{ name: "STEPPER1_POSITION_HI", address: 240, bank: "input", description: "Aktuelle Position Stepper 1, High", signed: true, display: "decimal" },
			{ name: "STEPPER1_POSITION_LO", address: 241, bank: "input", description: "Aktuelle Position Stepper 1, Low", display: "decimal" },
			{ name: "STEPPER1_STATUS", address: 242, bank: "input", description: "Bit0 Enabled, Bit1 Moving, Bit2 Error/StallGuard, Bit3 InPosition", display: "binary" },
			{ name: "STEPPER2_POSITION_HI", address: 243, bank: "input", description: "Aktuelle Position Stepper 2, High", signed: true, display: "decimal" },
			{ name: "STEPPER2_POSITION_LO", address: 244, bank: "input", description: "Aktuelle Position Stepper 2, Low", display: "decimal" },
			{ name: "STEPPER2_STATUS", address: 245, bank: "input", description: "Bit0 Enabled, Bit1 Moving, Bit2 Error/StallGuard, Bit3 InPosition", display: "binary" }
		]
	},
	{
		title: "Pneumatikventile",
		registers: [
			{ name: "VALVE1", address: 20, bank: "holding", description: "Pneumatikventil 1", gpio: "PD10", display: "bool" },
			{ name: "VALVE2", address: 21, bank: "holding", description: "Pneumatikventil 2", gpio: "PD5", display: "bool" },
			{ name: "VALVE3", address: 22, bank: "holding", description: "Pneumatikventil 3", gpio: "PD11", display: "bool" }
		]
	},
	{
		title: "Motor-PWM",
		registers: [
			{ name: "CONVEYOR_PWM", address: 40, bank: "holding", description: "Foerderband-Motor Duty", unit: "Promille", gpio: "TIM4_CH3/PD14", min: 0, max: 1000, display: "range" },
			{ name: "COMPRESSOR_PWM", address: 41, bank: "holding", description: "Kompressor-Motor Duty", unit: "Promille", gpio: "TIM4_CH4/PD15", min: 0, max: 1000, display: "range" }
		]
	},
	{
		title: "Stepper-Steuerung",
		registers: [
			{ name: "STEPPER_ENABLE", address: 60, bank: "holding", description: "Stepper-Treiber global freigeben", gpio: "PB7", display: "bool" },
			{ name: "STEPPER1_TARGET_HI", address: 61, bank: "holding", description: "Zielposition Stepper 1, High", signed: true, display: "decimal" },
			{ name: "STEPPER1_TARGET_LO", address: 62, bank: "holding", description: "Zielposition Stepper 1, Low", display: "decimal" },
			{ name: "STEPPER1_SPEED", address: 63, bank: "holding", description: "Geschwindigkeit Stepper 1", unit: "Schritte/s", display: "decimal" },
			{ name: "STEPPER1_ACCEL", address: 64, bank: "holding", description: "Beschleunigung Stepper 1", unit: "Schritte/s^2", display: "decimal" },
			{ name: "STEPPER2_TARGET_HI", address: 65, bank: "holding", description: "Zielposition Stepper 2, High", signed: true, display: "decimal" },
			{ name: "STEPPER2_TARGET_LO", address: 66, bank: "holding", description: "Zielposition Stepper 2, Low", display: "decimal" },
			{ name: "STEPPER2_SPEED", address: 67, bank: "holding", description: "Geschwindigkeit Stepper 2", unit: "Schritte/s", display: "decimal" },
			{ name: "STEPPER2_ACCEL", address: 68, bank: "holding", description: "Beschleunigung Stepper 2", unit: "Schritte/s^2", display: "decimal" },
			{ name: "STEPPER1_START_HOMING", address: 69, bank: "holding", description: "1=Homing starten (stoppt Stepper1, sensorloses Homing per StallGuard), 0=erfolgreich beendet, >1=Fehlercode", min: 0, max: 255, display: "decimal" },
			{ name: "STEPPER2_START_HOMING", address: 70, bank: "holding", description: "1=Homing starten (stoppt Stepper2, sensorloses Homing per StallGuard), 0=erfolgreich beendet, >1=Fehlercode", min: 0, max: 255, display: "decimal" }
		]
	},
	{
		title: "WS2812",
		registers: [
			{ name: "WS2812_CH1_PATTERN", address: 100, bank: "holding", description: "Index in fest im Code hinterlegte Farbmuster-Tabelle", gpio: "PE5/TIM15_CH1", display: "decimal" },
			{ name: "WS2812_CH2_PATTERN", address: 101, bank: "holding", description: "Index in fest im Code hinterlegte Farbmuster-Tabelle", gpio: "PE6/TIM15_CH2", display: "decimal" }
		]
	}
];

// Muss mit ModbusRegisters::INPUT_REGISTER_MAX_INDEX / HOLDING_REGISTER_MAX_INDEX
// (generated/modbus_register_map.inc) uebereinstimmen -- bestimmt, wie viele Werte
// /api/registers liefert.
export const INPUT_REGISTER_COUNT = 270; // Index 0..269
export const HOLDING_REGISTER_COUNT = 110; // Index 0..109
