#pragma once

#include <cstdint>

// Register-Map fuer die Control-Unit-Sensoren/Aktuatoren.
// Adressen sind Offsets in ModbusTcpServer::m_registers.{input,holding}_registers.
// 32-Bit-Werte belegen zwei Register (High-Word zuerst), siehe MbapHeader::serialize.
//
// Blockweise organisiert mit Reserve fuer spaetere Erweiterung. Nicht alle Bloecke
// sind bereits mit echter Peripherie hinterlegt (siehe Plan: gestuftes Vorgehen,
// Stage 1 deckt Diagnostik, Ethernet-Link, digitale I/O und den Drucksensor ab).
//
// Hinweis: Block-Markierungen heissen bewusst *_REGION_START/*_REGION_END statt
// *_BASE/*_END, da z.B. ETH_BASE/PWR_BASE bereits als CMSIS-Peripherie-Basisadressen
// (stm32h573xx.h) vergeben sind und ein Namenskollision einen Preprocessor-Fehler ausloest.

namespace ModbusRegisters {

// ---------------------------------------------------------------------------
// Input-Register (FC04, read-only)
// ---------------------------------------------------------------------------

namespace Input {

    // Diagnostik (Region-Start 0, Reserve bis 49)
    constexpr uint16_t HEALTH_STATE      = 0;
    constexpr uint16_t CHIP_ID_W0_HI     = 1;
    constexpr uint16_t CHIP_ID_W0_LO     = 2;
    constexpr uint16_t CHIP_ID_W1_HI     = 3;
    constexpr uint16_t CHIP_ID_W1_LO     = 4;
    constexpr uint16_t CHIP_ID_W2_HI     = 5;
    constexpr uint16_t CHIP_ID_W2_LO     = 6;
    constexpr uint16_t FW_VERSION_MAJOR  = 7;
    constexpr uint16_t FW_VERSION_MINOR  = 8;
    constexpr uint16_t FW_VERSION_PATCH  = 9;
    constexpr uint16_t TIMER_TICK        = 10; // freilaufender Systick-Zaehler, Ueberlauf nach 65535
    constexpr uint16_t DIAGNOSTICS_REGION_START = 0;
    constexpr uint16_t DIAGNOSTICS_REGION_END   = 49;

    // CAN-Bus-Statistik (Stage 4)
    constexpr uint16_t CAN_TX_COUNT_HI   = 50;
    constexpr uint16_t CAN_TX_COUNT_LO   = 51;
    constexpr uint16_t CAN_RX_COUNT_HI   = 52;
    constexpr uint16_t CAN_RX_COUNT_LO   = 53;
    constexpr uint16_t CAN_ERROR_COUNT_HI = 54;
    constexpr uint16_t CAN_ERROR_COUNT_LO = 55;
    constexpr uint16_t CAN_LAST_ERROR    = 56;
    constexpr uint16_t CAN_BUS_STATE     = 57;
    constexpr uint16_t CAN_REGION_START  = 50;
    constexpr uint16_t CAN_REGION_END    = 79;

    // Ethernet-Statistik (LAN8742) -- Link-Status ist bereits in Stage 1 real nutzbar
    constexpr uint16_t ETH_LINK_STATUS   = 80; // 0 = down, 1 = up
    constexpr uint16_t ETH_LINK_SPEED    = 81; // 0=10M, 1=100M, 2=1000M
    constexpr uint16_t ETH_LINK_DUPLEX   = 82; // 0=half, 1=full
    constexpr uint16_t ETH_TX_COUNT_HI   = 83;
    constexpr uint16_t ETH_TX_COUNT_LO   = 84;
    constexpr uint16_t ETH_RX_COUNT_HI   = 85;
    constexpr uint16_t ETH_RX_COUNT_LO   = 86;
    constexpr uint16_t ETH_ERROR_COUNT_HI = 87;
    constexpr uint16_t ETH_ERROR_COUNT_LO = 88;
    constexpr uint16_t ETH_LAST_ERROR    = 89;
    constexpr uint16_t ETH_REGION_START  = 80;
    constexpr uint16_t ETH_REGION_END    = 109;

    // Stromversorgung: INA226 @ I2C_4 + USB-PD (Stage 2)
    constexpr uint16_t PWR_BUS_VOLTAGE_MV     = 110;
    constexpr uint16_t PWR_SHUNT_VOLTAGE_UV   = 111; // signed
    constexpr uint16_t PWR_CURRENT_MA         = 112; // signed
    constexpr uint16_t PWR_POWER_MW           = 113;
    constexpr uint16_t PWR_PD_VOLTAGE_MV      = 114;
    constexpr uint16_t PWR_PD_CURRENT_MA      = 115;
    constexpr uint16_t PWR_PD_STATUS          = 116;
    constexpr uint16_t PWR_REGION_START       = 110;
    constexpr uint16_t PWR_REGION_END         = 139;

    // ToF-Sensoren VL53L0X (Stage 2)
    constexpr uint16_t TOF1_DISTANCE_MM  = 140; // I2C_1, IRQ PC15
    constexpr uint16_t TOF1_STATUS       = 141;
    constexpr uint16_t TOF2_DISTANCE_MM  = 142; // I2C_2, IRQ PH1
    constexpr uint16_t TOF2_STATUS       = 143;
    constexpr uint16_t TOF3_DISTANCE_MM  = 144; // I2C_4, IRQ PA3
    constexpr uint16_t TOF3_STATUS       = 145;
    constexpr uint16_t TOF_REGION_START  = 140;
    constexpr uint16_t TOF_REGION_END    = 159;

    // Farbsensor TCS34725 @ I2C_1, IRQ PH0 (Stage 2)
    constexpr uint16_t COLOR_CLEAR       = 160;
    constexpr uint16_t COLOR_RED         = 161;
    constexpr uint16_t COLOR_GREEN       = 162;
    constexpr uint16_t COLOR_BLUE        = 163;
    constexpr uint16_t COLOR_REGION_START = 160;
    constexpr uint16_t COLOR_REGION_END   = 179;

    // Lichtschranken -- LS1/LS2 real in Stage 1, LS3 (PC8) auf Eval-Board unverdrahtet
    constexpr uint16_t LIGHTBARRIER1     = 180; // PC6
    constexpr uint16_t LIGHTBARRIER2     = 181; // PC7
    constexpr uint16_t LIGHTBARRIER3     = 182; // PC8, real board only
    constexpr uint16_t LIGHTBARRIER_REGION_START = 180;
    constexpr uint16_t LIGHTBARRIER_REGION_END   = 199;

    // Analoger Drucksensor (ADC1 CH10, PA4) -- real in Stage 1, Rohwert
    constexpr uint16_t PRESSURE_RAW      = 200;
    constexpr uint16_t PRESSURE_REGION_START = 200;
    constexpr uint16_t PRESSURE_REGION_END   = 219;

    // Waegezelle HX711 (SPI2, PC2/PC3) (Stage 4)
    constexpr uint16_t SCALE_A_WEIGHT_HI = 220; // signed 32-bit
    constexpr uint16_t SCALE_A_WEIGHT_LO = 221;
    constexpr uint16_t SCALE_A_STATUS    = 222;
    constexpr uint16_t SCALE_B_RAW_HI    = 223; // Reserve/Erweiterung
    constexpr uint16_t SCALE_B_RAW_LO    = 224;
    constexpr uint16_t SCALE_B_STATUS    = 225;
    constexpr uint16_t SCALE_REGION_START = 220;
    constexpr uint16_t SCALE_REGION_END   = 239;

    // Stepper-Status (Stage 3)
    constexpr uint16_t STEPPER1_POSITION_HI = 240; // signed 32-bit
    constexpr uint16_t STEPPER1_POSITION_LO = 241;
    constexpr uint16_t STEPPER1_STATUS      = 242; // Bit0 Enabled, Bit1 Moving, Bit2 Error/StallGuard, Bit3 InPosition
    constexpr uint16_t STEPPER2_POSITION_HI = 243;
    constexpr uint16_t STEPPER2_POSITION_LO = 244;
    constexpr uint16_t STEPPER2_STATUS      = 245;
    constexpr uint16_t STEPPER_STATUS_REGION_START = 240;
    constexpr uint16_t STEPPER_STATUS_REGION_END   = 269;

} // namespace Input

// HealthState Bitfeld (Input::HEALTH_STATE)
namespace HealthBit {
    constexpr uint16_t OK               = 0;
    constexpr uint16_t OVERTEMPERATURE  = 1;
    constexpr uint16_t POWER_FAULT      = 2;
    constexpr uint16_t CAN_ERROR        = 3;
    constexpr uint16_t ETH_LINK_DOWN    = 4;
    constexpr uint16_t I2C_SENSOR_FAULT = 5;
    constexpr uint16_t STEPPER_FAULT    = 6;
}

// ---------------------------------------------------------------------------
// Holding-Register (FC03/06/16, read/write)
// ---------------------------------------------------------------------------

namespace Holding {

    // reserviert fuer kuenftige Konfiguration (Watchdog etc.)
    constexpr uint16_t RESERVED_CONFIG_REGION_START = 0;
    constexpr uint16_t RESERVED_CONFIG_REGION_END   = 19;

    // Pneumatikventile -- real in Stage 1
    constexpr uint16_t VALVE1            = 20; // PD10
    constexpr uint16_t VALVE2            = 21; // PD5
    constexpr uint16_t VALVE3            = 22; // PD11
    constexpr uint16_t VALVE_REGION_START = 20;
    constexpr uint16_t VALVE_REGION_END   = 39;

    // Motor-PWM (Stage 3), 0..1000 Promille Duty
    constexpr uint16_t CONVEYOR_PWM      = 40; // TIM4_CH3/PD14
    constexpr uint16_t COMPRESSOR_PWM    = 41; // TIM4_CH4/PD15
    constexpr uint16_t MOTOR_REGION_START = 40;
    constexpr uint16_t MOTOR_REGION_END   = 59;

    // Stepper-Steuerung (Stage 3)
    constexpr uint16_t STEPPER_ENABLE       = 60; // PB7, global
    constexpr uint16_t STEPPER1_TARGET_HI   = 61; // signed 32-bit
    constexpr uint16_t STEPPER1_TARGET_LO   = 62;
    constexpr uint16_t STEPPER1_SPEED       = 63; // Schritte/s
    constexpr uint16_t STEPPER1_ACCEL       = 64; // Schritte/s^2
    constexpr uint16_t STEPPER2_TARGET_HI   = 65;
    constexpr uint16_t STEPPER2_TARGET_LO   = 66;
    constexpr uint16_t STEPPER2_SPEED       = 67;
    constexpr uint16_t STEPPER2_ACCEL       = 68;
    constexpr uint16_t STEPPER_CTRL_REGION_START = 60;
    constexpr uint16_t STEPPER_CTRL_REGION_END   = 99;

    // WS2812 (Stage 3): Index in fest im Code hinterlegte Farbmuster-Tabelle
    constexpr uint16_t WS2812_CH1_PATTERN   = 100; // PE5/TIM15_CH1
    constexpr uint16_t WS2812_CH2_PATTERN   = 101; // PE6/TIM15_CH2
    constexpr uint16_t WS2812_REGION_START  = 100;
    constexpr uint16_t WS2812_REGION_END    = 109;

} // namespace Holding

} // namespace ModbusRegisters
