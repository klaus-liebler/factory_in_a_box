#pragma once
// ============================================================================
// WS2812 -- setzt WS2812_1/WS2812_2 (PE5/PE6, TIM15 CH1/CH2) anhand des jeweiligen
// Pattern-Index-Holding-Registers auf eine feste Farbe aus einer kleinen, im Code hinterlegten
// Palette. Zeigt Show() nur bei tatsaechlicher Indexaenderung erneut (vermeidet unnoetige
// DMA-Transfers).
// ============================================================================
#include "interfaces.hh"
#include "modbus_register_model.hh"
#include "ws2812.hpp"

extern "C" TIM_HandleTypeDef htim15;

class Ws2812SetupAndLoop : public ISetupAndLoop {
    private:
    // TIM15 haengt am APB2-Timertakt: SystemClock_Config() (main.c) -> APB2TimFreq_Value=160000000
    // im .ioc (Timer-Takt verdoppelt sich ggue. dem 80-MHz-APB2-Takt, da dessen Prescaler != 1
    // ist). Als Template-Parameter direkt hier an der Instanziierungsstelle -- siehe ws2812.hpp
    // fuer die Begruendung, warum das kein interner Konstantenwert der Bibliothek ist.
    // CH1: 8er-Kette (WS2812_1-Anschluss), CH2 (weiterhin) eine einzelne LED.
    static constexpr uint32_t CH1_LED_COUNT = 8;

    // Palette fuer WS2812_CH2_PATTERN -- Index 0 = aus, sonst einfache Volltonfarben.
    static constexpr uint32_t CH2_PALETTE[] = {
        0x000000, // 0: aus
        0xFF0000, // 1: rot
        0x00FF00, // 2: gruen
        0x0000FF, // 3: blau
        0xFFFFFF, // 4: weiss
        0xFFFF00, // 5: gelb
        0x00FFFF, // 6: cyan
        0xFF00FF, // 7: magenta
    };
    static constexpr uint16_t CH2_PALETTE_SIZE = sizeof(CH2_PALETTE) / sizeof(CH2_PALETTE[0]);

    Modbus::IModbusRegisterModel& register_model;
    ws2812::Driver<160000000UL, CH1_LED_COUNT> ch1_driver;
    ws2812::Driver<160000000UL> ch2_driver;

    uint16_t last_pattern1_ = UINT16_MAX;
    uint16_t last_pattern2_ = UINT16_MAX;

    // Muster fuer WS2812_CH1_PATTERN (8er-Kette): 0=aus, 1=rot, 2=gelb, 3=gruen,
    // 4=abwechselnd blau/weiss. Kein definiertes "Blink"/"Chase"-Muster: ShowEach() setzt nur ein
    // statisches Farbmuster pro Aufruf, fuer echte Animationen muesste eine Zeit-/Frame-Logik
    // hinzukommen.
    static void fill_ch1_pattern(uint16_t pattern, uint32_t (&out)[CH1_LED_COUNT]) {
        switch (pattern % 5) {
            default:
            case 0: // aus
                for (uint32_t& c : out) c = 0x000000;
                break;
            case 1: // rot
                for (uint32_t& c : out) c = 0xFF0000;
                break;
            case 2: // gelb
                for (uint32_t& c : out) c = 0xFFFF00;
                break;
            case 3: // gruen
                for (uint32_t& c : out) c = 0x00FF00;
                break;
            case 4: // abwechselnd blau/weiss
                for (uint32_t i = 0; i < CH1_LED_COUNT; ++i) {
                    out[i] = (i % 2 == 0) ? 0x0000FF : 0xFFFFFF;
                }
                break;
        }
    }

    public:
    Ws2812SetupAndLoop(Modbus::IModbusRegisterModel& model) : register_model(model) {}

    void Setup() override {
        ch1_driver.Init(&htim15, TIM_CHANNEL_1);
        // use_update_burst=true: TIM15 hat auf STM32H5 keine eigene DMA-Request-Leitung fuer CH2
        // (weder GPDMA1 noch GPDMA2 bieten "TIM15_CH2" an, nur CH1/UP/TRIG/COM -- deshalb in
        // CubeMX fuer CH2 auch nicht auswaehlbar) -- CH2 laeuft daher ueber die TIM15_UP-Leitung
        // per DMA-Burst-Adressierung auf CCR2, s. ws2812.hpp fuer Details. In CubeMX braucht CH2
        // dafuer einen GPDMA-Request auf "TIM15_UP" (nicht "TIM15_CH2"!) statt auf einen
        // Kanal-Request.
        ch2_driver.Init(&htim15, TIM_CHANNEL_2, /*use_update_burst=*/true);
    }

    void Loop(uint32_t now) override {
        (void)now;
        uint16_t pattern1 = register_model.GetHoldingRegister(ModbusRegisters::Holding::WS2812_CH1_PATTERN);
        uint16_t pattern2 = register_model.GetHoldingRegister(ModbusRegisters::Holding::WS2812_CH2_PATTERN);

        if (pattern1 != last_pattern1_) {
            uint32_t colors[CH1_LED_COUNT];
            fill_ch1_pattern(pattern1, colors);
            ch1_driver.ShowEach(colors);
            last_pattern1_ = pattern1;
        }
        if (pattern2 != last_pattern2_) {
            ch2_driver.Show(CH2_PALETTE[pattern2 % CH2_PALETTE_SIZE]);
            last_pattern2_ = pattern2;
        }
    }
};
