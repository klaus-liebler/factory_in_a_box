// ============================================================================
// WS2812 -- setzt WS2812_1/WS2812_2 (PE5/PE6, TIM15 CH1/CH2) anhand des jeweiligen
// Pattern-Index-Holding-Registers auf eine feste Farbe aus einer kleinen, im Code hinterlegten
// Palette. Zeigt Show() nur bei tatsaechlicher Indexaenderung erneut (vermeidet unnoetige
// DMA-Transfers).
// ============================================================================
#include "ws2812_control.hpp"
#include "app_state.hpp"
#include "modbus_register_map.hpp"
#include "ws2812.hpp"

extern "C" TIM_HandleTypeDef htim15;

// TIM15 haengt am APB2-Timertakt: SystemClock_Config() (main.c) -> APB2TimFreq_Value=160000000
// im .ioc (Timer-Takt verdoppelt sich ggue. dem 80-MHz-APB2-Takt, da dessen Prescaler != 1
// ist). Als Template-Parameter direkt hier an der Instanziierungsstelle -- siehe ws2812.hpp
// fuer die Begruendung, warum das kein interner Konstantenwert der Bibliothek ist.
static ws2812::Driver<160000000UL> ch1_driver;
static ws2812::Driver<160000000UL> ch2_driver;

// Palette fuer WS2812_CH{1,2}_PATTERN -- Index 0 = aus, sonst einfache Volltonfarben.
// Kein definiertes "Blink"/"Chase"-Muster: Show() setzt nur einen statischen Farbwert pro
// Aufruf, fuer echte Animationen muesste diese Tabelle durch eine Zeit-/Frame-Logik ersetzt
// werden.
static constexpr uint32_t PALETTE[] = {
    0x000000, // 0: aus
    0xFF0000, // 1: rot
    0x00FF00, // 2: gruen
    0x0000FF, // 3: blau
    0xFFFFFF, // 4: weiss
    0xFFFF00, // 5: gelb
    0x00FFFF, // 6: cyan
    0xFF00FF, // 7: magenta
};
constexpr uint16_t PALETTE_SIZE = sizeof(PALETTE) / sizeof(PALETTE[0]);

void ws2812_setup() {
    ch1_driver.Init(&htim15, TIM_CHANNEL_1);
    // use_update_burst=true: TIM15 hat auf STM32H5 keine eigene DMA-Request-Leitung fuer CH2
    // (weder GPDMA1 noch GPDMA2 bieten "TIM15_CH2" an, nur CH1/UP/TRIG/COM -- deshalb in
    // CubeMX fuer CH2 auch nicht auswaehlbar) -- CH2 laeuft daher ueber die TIM15_UP-Leitung
    // per DMA-Burst-Adressierung auf CCR2, s. ws2812.hpp fuer Details. In CubeMX braucht CH2
    // dafuer einen GPDMA-Request auf "TIM15_UP" (nicht "TIM15_CH2"!) statt auf einen
    // Kanal-Request.
    ch2_driver.Init(&htim15, TIM_CHANNEL_2, /*use_update_burst=*/true);
}

void ws2812_update() {
    // static: Zustand muss zwischen einzelnen update()-Aufrufen erhalten bleiben (kein eigener
    // Thread mit persistentem Stack-Frame mehr, s. io_thread.cpp).
    static uint16_t last_pattern1 = UINT16_MAX;
    static uint16_t last_pattern2 = UINT16_MAX;

    ModbusTcpServer& server = *g_app_state.modbus_server;

    uint16_t pattern1 = server.read_holding_register(ModbusRegisters::Holding::WS2812_CH1_PATTERN);
    uint16_t pattern2 = server.read_holding_register(ModbusRegisters::Holding::WS2812_CH2_PATTERN);

    if (pattern1 != last_pattern1) {
        ch1_driver.Show(PALETTE[pattern1 % PALETTE_SIZE]);
        last_pattern1 = pattern1;
    }
    if (pattern2 != last_pattern2) {
        ch2_driver.Show(PALETTE[pattern2 % PALETTE_SIZE]);
        last_pattern2 = pattern2;
    }
}
