// GENERIERT von tools/generate-register-map.mjs aus register-map.json -- nicht von Hand
// editieren. Aenderungen an der Register-Map gehoeren in register-map.json, anschliessend
// "node tools/generate-register-map.mjs" erneut ausfuehren.
//
// Nur per #include aus Core/Src/modbus_register_model.hh eingebunden (innerhalb des dort
// bereits geoeffneten namespace-Blocks), nicht eigenstaendig uebersetzbar -- enthaelt bewusst
// nur constexpr-Zeilen, keine eigene namespace-Deklaration/Klammern.
//
// Adressen sind Offsets in ModbusRegisterModel::holding_registers_.
// 32-Bit-Werte belegen zwei Register (High-Word zuerst), siehe MbapHeader::serialize.

// Reserviert (Region-Start 0, Reserve bis 19)
constexpr uint16_t RESERVED_CONFIG_REGION_START = 0;
constexpr uint16_t RESERVED_CONFIG_REGION_END   = 19;

// Pneumatikventile (Region-Start 20, Reserve bis 39)
constexpr uint16_t VALVE1 = 20; // Pneumatikventil 1 -- PD10
constexpr uint16_t VALVE2 = 21; // Pneumatikventil 2 -- PD5
constexpr uint16_t VALVE3 = 22; // Pneumatikventil 3 -- PD11
constexpr uint16_t VALVE_REGION_START = 20;
constexpr uint16_t VALVE_REGION_END   = 39;

// Motor-PWM (Region-Start 40, Reserve bis 59)
constexpr uint16_t CONVEYOR_PWM = 40; // Foerderband-Motor Duty -- [Promille] -- TIM4_CH3/PD14
constexpr uint16_t COMPRESSOR_PWM = 41; // Kompressor-Motor Duty -- [Promille] -- TIM4_CH4/PD15
constexpr uint16_t MOTOR_REGION_START = 40;
constexpr uint16_t MOTOR_REGION_END   = 59;

// Stepper-Steuerung (Region-Start 60, Reserve bis 99)
constexpr uint16_t STEPPER_ENABLE = 60; // Stepper-Treiber global freigeben -- PB7
constexpr uint16_t STEPPER1_TARGET_HI = 61; // Zielposition Stepper 1, High -- signed
constexpr uint16_t STEPPER1_TARGET_LO = 62; // Zielposition Stepper 1, Low
constexpr uint16_t STEPPER1_SPEED = 63; // Geschwindigkeit Stepper 1 -- [Schritte/s]
constexpr uint16_t STEPPER1_ACCEL = 64; // Beschleunigung Stepper 1 -- [Schritte/s^2]
constexpr uint16_t STEPPER2_TARGET_HI = 65; // Zielposition Stepper 2, High -- signed
constexpr uint16_t STEPPER2_TARGET_LO = 66; // Zielposition Stepper 2, Low
constexpr uint16_t STEPPER2_SPEED = 67; // Geschwindigkeit Stepper 2 -- [Schritte/s]
constexpr uint16_t STEPPER2_ACCEL = 68; // Beschleunigung Stepper 2 -- [Schritte/s^2]
constexpr uint16_t STEPPER1_START_HOMING = 69; // 1=Homing starten (stoppt Stepper1, sensorloses Homing per StallGuard), 0=erfolgreich beendet, >1=Fehlercode
constexpr uint16_t STEPPER2_START_HOMING = 70; // 1=Homing starten (stoppt Stepper2, sensorloses Homing per StallGuard), 0=erfolgreich beendet, >1=Fehlercode
constexpr uint16_t STEPPER_CTRL_REGION_START = 60;
constexpr uint16_t STEPPER_CTRL_REGION_END   = 99;

// WS2812 (Region-Start 100, Reserve bis 109)
constexpr uint16_t WS2812_CH1_PATTERN = 100; // Index in fest im Code hinterlegte Farbmuster-Tabelle -- PE5/TIM15_CH1
constexpr uint16_t WS2812_CH2_PATTERN = 101; // Index in fest im Code hinterlegte Farbmuster-Tabelle -- PE6/TIM15_CH2
constexpr uint16_t WS2812_REGION_START = 100;
constexpr uint16_t WS2812_REGION_END   = 109;

