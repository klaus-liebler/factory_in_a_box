// GENERIERT von tools/generate-register-map.mjs aus register-map.json -- nicht von Hand
// editieren. Aenderungen an der Register-Map gehoeren in register-map.json, anschliessend
// "node tools/generate-register-map.mjs" erneut ausfuehren.

export type RegisterBank = "input" | "holding";
export type RegisterControl = "readonly" | "toggle" | "slider" | "number";

export interface RegisterDef {
	name: string;
	address: number;
	bank: RegisterBank;
	control: RegisterControl;
	description?: string;
	unit?: string;
	signed?: boolean;
	gpio?: string;
	i2c?: { bus: string; irqPin?: string };
	min?: number;
	max?: number;
}

export interface RegisterRegion {
	title: string;
	registers: RegisterDef[];
}

export const REGIONS: RegisterRegion[] = [
	{
		title: "Diagnostik",
		registers: [
			{ name: "HEALTH_STATE", address: 0, bank: "input", description: "Health-Bitfeld (siehe healthBits)", control: "readonly" },
			{ name: "CHIP_ID_W0_HI", address: 1, bank: "input", description: "Chip-UID Wort 0, High", control: "readonly" },
			{ name: "CHIP_ID_W0_LO", address: 2, bank: "input", description: "Chip-UID Wort 0, Low", control: "readonly" },
			{ name: "CHIP_ID_W1_HI", address: 3, bank: "input", description: "Chip-UID Wort 1, High", control: "readonly" },
			{ name: "CHIP_ID_W1_LO", address: 4, bank: "input", description: "Chip-UID Wort 1, Low", control: "readonly" },
			{ name: "CHIP_ID_W2_HI", address: 5, bank: "input", description: "Chip-UID Wort 2, High", control: "readonly" },
			{ name: "CHIP_ID_W2_LO", address: 6, bank: "input", description: "Chip-UID Wort 2, Low", control: "readonly" },
			{ name: "FW_VERSION_MAJOR", address: 7, bank: "input", description: "Firmware-Version Major", control: "readonly" },
			{ name: "FW_VERSION_MINOR", address: 8, bank: "input", description: "Firmware-Version Minor", control: "readonly" },
			{ name: "FW_VERSION_PATCH", address: 9, bank: "input", description: "Firmware-Version Patch", control: "readonly" },
			{ name: "TIMER_TICK", address: 10, bank: "input", description: "Freilaufender Systick-Zaehler, Ueberlauf nach 65535", unit: "ticks", control: "readonly" }
		]
	},
	{
		title: "CAN-Bus",
		registers: [
			{ name: "CAN_TX_COUNT_HI", address: 50, bank: "input", description: "Gesendete CAN-Frames, High", control: "readonly" },
			{ name: "CAN_TX_COUNT_LO", address: 51, bank: "input", description: "Gesendete CAN-Frames, Low", control: "readonly" },
			{ name: "CAN_RX_COUNT_HI", address: 52, bank: "input", description: "Empfangene CAN-Frames, High", control: "readonly" },
			{ name: "CAN_RX_COUNT_LO", address: 53, bank: "input", description: "Empfangene CAN-Frames, Low", control: "readonly" },
			{ name: "CAN_ERROR_COUNT_HI", address: 54, bank: "input", description: "CAN-Fehlerzaehler, High", control: "readonly" },
			{ name: "CAN_ERROR_COUNT_LO", address: 55, bank: "input", description: "CAN-Fehlerzaehler, Low", control: "readonly" },
			{ name: "CAN_LAST_ERROR", address: 56, bank: "input", description: "Letzter CAN-Fehlercode", control: "readonly" },
			{ name: "CAN_BUS_STATE", address: 57, bank: "input", description: "CAN-Bus-Zustand", control: "readonly" }
		]
	},
	{
		title: "Ethernet",
		registers: [
			{ name: "ETH_LINK_STATUS", address: 80, bank: "input", description: "0 = down, 1 = up", control: "readonly" },
			{ name: "ETH_LINK_SPEED", address: 81, bank: "input", description: "0=10M, 1=100M, 2=1000M", control: "readonly" },
			{ name: "ETH_LINK_DUPLEX", address: 82, bank: "input", description: "0=half, 1=full", control: "readonly" },
			{ name: "ETH_TX_COUNT_HI", address: 83, bank: "input", description: "Gesendete Ethernet-Frames, High", control: "readonly" },
			{ name: "ETH_TX_COUNT_LO", address: 84, bank: "input", description: "Gesendete Ethernet-Frames, Low", control: "readonly" },
			{ name: "ETH_RX_COUNT_HI", address: 85, bank: "input", description: "Empfangene Ethernet-Frames, High", control: "readonly" },
			{ name: "ETH_RX_COUNT_LO", address: 86, bank: "input", description: "Empfangene Ethernet-Frames, Low", control: "readonly" },
			{ name: "ETH_ERROR_COUNT_HI", address: 87, bank: "input", description: "Ethernet-Fehlerzaehler, High", control: "readonly" },
			{ name: "ETH_ERROR_COUNT_LO", address: 88, bank: "input", description: "Ethernet-Fehlerzaehler, Low", control: "readonly" },
			{ name: "ETH_LAST_ERROR", address: 89, bank: "input", description: "Letzter Ethernet/PHY-Fehlercode", control: "readonly" }
		]
	},
	{
		title: "Stromversorgung",
		registers: [
			{ name: "PWR_BUS_VOLTAGE_MV", address: 110, bank: "input", description: "Busspannung (INA226)", unit: "mV", control: "readonly" },
			{ name: "PWR_SHUNT_VOLTAGE_UV", address: 111, bank: "input", description: "Shunt-Spannung (INA226)", unit: "uV", signed: true, control: "readonly" },
			{ name: "PWR_CURRENT_MA", address: 112, bank: "input", description: "Stromaufnahme (INA226)", unit: "mA", signed: true, control: "readonly" },
			{ name: "PWR_POWER_MW", address: 113, bank: "input", description: "Leistungsaufnahme (INA226)", unit: "mW", control: "readonly" },
			{ name: "PWR_PD_VOLTAGE_MV", address: 114, bank: "input", description: "Aktiv ausgehandelte USB-PD-Spannung, PDSink::activeVoltage", unit: "mV", control: "readonly" },
			{ name: "PWR_PD_CURRENT_MA", address: 115, bank: "input", description: "Aktiv ausgehandelter USB-PD-Maximalstrom, PDSink::activeCurrent", unit: "mA", control: "readonly" },
			{ name: "PWR_PD_STATUS", address: 116, bank: "input", description: "0 = keine Quelle verbunden, 1 = verbunden/Kontrakt aktiv", control: "readonly" }
		]
	},
	{
		title: "ToF-Sensoren",
		registers: [
			{ name: "TOF1_DISTANCE_MM", address: 140, bank: "input", description: "Abstand Sensor 1 (VL53L0X)", unit: "mm", i2c: { bus: "I2C_1", irqPin: "PC15" }, control: "readonly" },
			{ name: "TOF1_STATUS", address: 141, bank: "input", description: "Status Sensor 1", i2c: { bus: "I2C_1", irqPin: "PC15" }, control: "readonly" },
			{ name: "TOF2_DISTANCE_MM", address: 142, bank: "input", description: "Abstand Sensor 2 (VL53L0X)", unit: "mm", i2c: { bus: "I2C_2", irqPin: "PH1" }, control: "readonly" },
			{ name: "TOF2_STATUS", address: 143, bank: "input", description: "Status Sensor 2", i2c: { bus: "I2C_2", irqPin: "PH1" }, control: "readonly" },
			{ name: "TOF3_DISTANCE_MM", address: 144, bank: "input", description: "Abstand Sensor 3 (VL53L0X)", unit: "mm", i2c: { bus: "I2C_4", irqPin: "PA3" }, control: "readonly" },
			{ name: "TOF3_STATUS", address: 145, bank: "input", description: "Status Sensor 3", i2c: { bus: "I2C_4", irqPin: "PA3" }, control: "readonly" }
		]
	},
	{
		title: "Farbsensor",
		registers: [
			{ name: "COLOR_CLEAR", address: 160, bank: "input", description: "Farbsensor TCS34725, Clear-Kanal", i2c: { bus: "I2C_1", irqPin: "PH0" }, control: "readonly" },
			{ name: "COLOR_RED", address: 161, bank: "input", description: "Farbsensor TCS34725, Rot-Kanal", i2c: { bus: "I2C_1", irqPin: "PH0" }, control: "readonly" },
			{ name: "COLOR_GREEN", address: 162, bank: "input", description: "Farbsensor TCS34725, Gruen-Kanal", i2c: { bus: "I2C_1", irqPin: "PH0" }, control: "readonly" },
			{ name: "COLOR_BLUE", address: 163, bank: "input", description: "Farbsensor TCS34725, Blau-Kanal", i2c: { bus: "I2C_1", irqPin: "PH0" }, control: "readonly" }
		]
	},
	{
		title: "Lichtschranken",
		registers: [
			{ name: "LIGHTBARRIER1", address: 180, bank: "input", description: "Lichtschranke 1 (NPN, aktiv=LOW)", gpio: "PC6", control: "readonly" },
			{ name: "LIGHTBARRIER2", address: 181, bank: "input", description: "Lichtschranke 2 (NPN, aktiv=LOW)", gpio: "PC7", control: "readonly" },
			{ name: "LIGHTBARRIER3", address: 182, bank: "input", description: "Lichtschranke 3 (NPN, aktiv=LOW) -- nur auf realem Board verdrahtet", gpio: "PC8", control: "readonly" }
		]
	},
	{
		title: "Drucksensor",
		registers: [
			{ name: "PRESSURE_RAW", address: 200, bank: "input", description: "Analoger Drucksensor, Rohwert (ADC1 CH10)", gpio: "PA4", control: "readonly" }
		]
	},
	{
		title: "Waegezelle",
		registers: [
			{ name: "SCALE_A_WEIGHT_HI", address: 220, bank: "input", description: "Gewicht Waegezelle A, High", signed: true, control: "readonly" },
			{ name: "SCALE_A_WEIGHT_LO", address: 221, bank: "input", description: "Gewicht Waegezelle A, Low", control: "readonly" },
			{ name: "SCALE_A_STATUS", address: 222, bank: "input", description: "Status Waegezelle A", control: "readonly" },
			{ name: "SCALE_B_RAW_HI", address: 223, bank: "input", description: "Rohwert Waegezelle B, High (Reserve/Erweiterung)", control: "readonly" },
			{ name: "SCALE_B_RAW_LO", address: 224, bank: "input", description: "Rohwert Waegezelle B, Low", control: "readonly" },
			{ name: "SCALE_B_STATUS", address: 225, bank: "input", description: "Status Waegezelle B", control: "readonly" }
		]
	},
	{
		title: "Stepper-Status",
		registers: [
			{ name: "STEPPER1_POSITION_HI", address: 240, bank: "input", description: "Aktuelle Position Stepper 1, High", signed: true, control: "readonly" },
			{ name: "STEPPER1_POSITION_LO", address: 241, bank: "input", description: "Aktuelle Position Stepper 1, Low", control: "readonly" },
			{ name: "STEPPER1_STATUS", address: 242, bank: "input", description: "Bit0 Enabled, Bit1 Moving, Bit2 Error/StallGuard, Bit3 InPosition", control: "readonly" },
			{ name: "STEPPER2_POSITION_HI", address: 243, bank: "input", description: "Aktuelle Position Stepper 2, High", signed: true, control: "readonly" },
			{ name: "STEPPER2_POSITION_LO", address: 244, bank: "input", description: "Aktuelle Position Stepper 2, Low", control: "readonly" },
			{ name: "STEPPER2_STATUS", address: 245, bank: "input", description: "Bit0 Enabled, Bit1 Moving, Bit2 Error/StallGuard, Bit3 InPosition", control: "readonly" }
		]
	},
	{
		title: "Pneumatikventile",
		registers: [
			{ name: "VALVE1", address: 20, bank: "holding", description: "Pneumatikventil 1", gpio: "PD10", control: "toggle", min: 0, max: 1 },
			{ name: "VALVE2", address: 21, bank: "holding", description: "Pneumatikventil 2", gpio: "PD5", control: "toggle", min: 0, max: 1 },
			{ name: "VALVE3", address: 22, bank: "holding", description: "Pneumatikventil 3", gpio: "PD11", control: "toggle", min: 0, max: 1 }
		]
	},
	{
		title: "Motor-PWM",
		registers: [
			{ name: "CONVEYOR_PWM", address: 40, bank: "holding", description: "Foerderband-Motor Duty", unit: "Promille", gpio: "TIM4_CH3/PD14", control: "slider", min: 0, max: 1000 },
			{ name: "COMPRESSOR_PWM", address: 41, bank: "holding", description: "Kompressor-Motor Duty", unit: "Promille", gpio: "TIM4_CH4/PD15", control: "slider", min: 0, max: 1000 }
		]
	},
	{
		title: "Stepper-Steuerung",
		registers: [
			{ name: "STEPPER_ENABLE", address: 60, bank: "holding", description: "Stepper-Treiber global freigeben", gpio: "PB7", control: "toggle", min: 0, max: 1 },
			{ name: "STEPPER1_TARGET_HI", address: 61, bank: "holding", description: "Zielposition Stepper 1, High", signed: true, control: "number" },
			{ name: "STEPPER1_TARGET_LO", address: 62, bank: "holding", description: "Zielposition Stepper 1, Low", control: "number" },
			{ name: "STEPPER1_SPEED", address: 63, bank: "holding", description: "Geschwindigkeit Stepper 1", unit: "Schritte/s", control: "number" },
			{ name: "STEPPER1_ACCEL", address: 64, bank: "holding", description: "Beschleunigung Stepper 1", unit: "Schritte/s^2", control: "number" },
			{ name: "STEPPER2_TARGET_HI", address: 65, bank: "holding", description: "Zielposition Stepper 2, High", signed: true, control: "number" },
			{ name: "STEPPER2_TARGET_LO", address: 66, bank: "holding", description: "Zielposition Stepper 2, Low", control: "number" },
			{ name: "STEPPER2_SPEED", address: 67, bank: "holding", description: "Geschwindigkeit Stepper 2", unit: "Schritte/s", control: "number" },
			{ name: "STEPPER2_ACCEL", address: 68, bank: "holding", description: "Beschleunigung Stepper 2", unit: "Schritte/s^2", control: "number" }
		]
	},
	{
		title: "WS2812",
		registers: [
			{ name: "WS2812_CH1_PATTERN", address: 100, bank: "holding", description: "Index in fest im Code hinterlegte Farbmuster-Tabelle", gpio: "PE5/TIM15_CH1", control: "number" },
			{ name: "WS2812_CH2_PATTERN", address: 101, bank: "holding", description: "Index in fest im Code hinterlegte Farbmuster-Tabelle", gpio: "PE6/TIM15_CH2", control: "number" }
		]
	}
];

// Muss mit ModbusRegisters::INPUT_REGISTER_MAX_INDEX / HOLDING_REGISTER_MAX_INDEX
// (modbus_register_map.hpp) uebereinstimmen -- bestimmt, wie viele Werte /api/registers
// liefert.
export const INPUT_REGISTER_COUNT = 270; // Index 0..269
export const HOLDING_REGISTER_COUNT = 110; // Index 0..109
