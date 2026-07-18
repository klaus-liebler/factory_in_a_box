#pragma once
// Minimaler WS2812(B)-Treiber ueber Timer-PWM + DMA: ein DMA-Puffer mit einem CCR-Wert pro
// Bit wird per DMA in das Compare-Register geschoben, wobei die Duty-Cycle-Breite pro Slot
// ueber T0H/T1H (0-/1-Bit-High-Zeit im 800-kHz-Protokoll) entscheidet -- die eigentliche
// Bit-Timing-Praezision kommt vom Timer/DMA, nicht von Software-Delays (unabhaengig vom
// CPU-Takt robust).
//
// TimerClockHz ist bewusst ein Template-Parameter statt einer intern berechneten/geratenen
// Konstante: der tatsaechliche Timer-Kerntakt haengt von der Clock-Tree-Konfiguration des
// jeweiligen Projekts ab (u.a. vom STM32-Fallstrick, dass der Timer-Takt sich verdoppelt,
// sobald der zugehoerige APB-Prescaler != 1 ist) und laesst sich nicht aus einer
// Chip-/Peripherie-Datenbank ableiten -- exakt dasselbe Prinzip wie bei
// stm32_libs/common_stm32/timer.hh's PwmOutput<..., TimerClockSourceHz>, siehe dortigen
// Kommentar. Der Vorteil ggue. einer freistehenden Konstante (wie vorher hier): der Wert steht
// jetzt direkt an der Instanziierungsstelle (siehe ws2812_control.cpp) und kann nicht mehr
// unbemerkt von der tatsaechlichen SystemClock_Config() abweichen, ohne dass ein
// static_assert anschlaegt -- genau das ist vorher schon einmal passiert (Konstante blieb auf
// 64 MHz stehen, nachdem der Systemtakt auf 160 MHz erhoeht wurde).
//
// ZWEI DMA-STRATEGIEN, waehlbar per Init()-Parameter use_update_burst:
//
// - use_update_burst=false (Default): DMA haengt am kanalspezifischen CCx-Request
//   (HAL_TIM_PWM_Start_DMA(), CubeMX: TIM "DMA Settings" -> Request fuer TIMx_CHy). Nicht
//   jeder Timer/Kanal hat dafuer aber ueberhaupt eine eigene DMA-Request-Leitung im Silizium
//   -- TIM15 CH2 z.B. hat auf STM32H5 KEINE eigene Request-Leitung (weder GPDMA1 noch GPDMA2
//   bieten "TIM15_CH2" an, nur TIM15_CH1/UP/TRIG/COM, s. stm32h5xx_hal_dma.h) und laesst sich
//   in CubeMX dafuer folgerichtig auch nicht auswaehlen.
// - use_update_burst=true: DMA haengt stattdessen am Update-Event (TIM15_UP, auf JEDEM Timer
//   verfuegbar) und schreibt die Werte per DMA-Burst-Adressierung (TIMx_DMAR/DCR) direkt in
//   das Compare-Register des gewuenschten Kanals (HAL_TIM_DMABurst_WriteStart(),
//   BurstLength=1 -- ein Register pro Update-Ereignis, funktional identisch zum
//   kanalspezifischen Weg). Das ist der einzig moegliche Weg fuer TIM15 CH2.
//
// Fuer beide Strategien: braucht eine per CubeMX auf die jeweilige Request-Leitung (TIM15_CH1
// bzw. TIM15_UP) verlinkte GPDMA-Instanz (CubeMX: TIM15 "DMA Settings"), sonst bleibt Show()
// wirkungslos (per Software-Check unten abgefangen, s. dort -- die HAL selbst wuerde bei
// HAL_TIM_PWM_Start_DMA() ohne verlinktes DMA sogar hardfaulten statt HAL_ERROR zu liefern,
// siehe Kommentar in Show()).
#include <cstdint>
#include <algorithm>
#include "main.h"
#include "log.h"
#include "hw_config_assert.hh"

namespace ws2812 {

// TimerClockHz: tatsaechlicher Timer-Eingangstakt in Hz (siehe Klassenkommentar oben).
// LedCount: Anzahl in Reihe geschalteter LEDs an diesem Kanal. Laut Schaltplan
// (pcb_factory_control_unit/actors.kicad_sch) sind WS2812_1/WS2812_2 als einzelne
// global_labels ohne erkennbare Weiterschaltung (DOUT->naechste DIN) vorhanden -- hier wird
// daher standardmaessig von genau einer LED pro Kanal ausgegangen. Bei einer tatsaechlichen
// Kette den Template-Parameter entsprechend erhoehen.
template <uint32_t TimerClockHz, uint32_t LedCount = 1>
class Driver {
public:
    static constexpr uint32_t WS2812_BIT_HZ = 800000UL;
    static constexpr uint32_t ARR = TimerClockHz / WS2812_BIT_HZ - 1;
    // T0H/T1H als Bruchteil der Bit-Periode (WS2812B-Datenblatt: T0H~0.35us von 1.25us, T1H~0.7us).
    static constexpr uint32_t CCR_BIT0 = (ARR + 1) * 35 / 100;
    static constexpr uint32_t CCR_BIT1 = (ARR + 1) * 70 / 100;

    static_assert(ARR > 0 && ARR <= 0xFFFF,
        "ws2812::Driver: TimerClockHz ergibt bei 800kHz Bit-Takt keinen gueltigen 16-Bit-ARR-Wert "
        "(zu niedriger oder zu hoher Takt) -- TimerClockHz pruefen (tatsaechlicher Timer-Takt, "
        "nicht zwingend der CPU-Kerntakt, siehe Klassenkommentar).");

    // use_update_burst: s. Klassenkommentar oben -- auf true setzen, wenn der gewaehlte Kanal
    // (z.B. TIM15 CH2) keine eigene DMA-Request-Leitung hat.
    void Init(TIM_HandleTypeDef *htim, uint32_t channel, bool use_update_burst = false) {
        htim_ = htim;
        channel_ = channel;
        use_update_burst_ = use_update_burst;
        configure_pwm_channel(htim_, channel_);
        __HAL_TIM_SET_AUTORELOAD(htim_, ARR);

        // Dieselbe Bedingung, die ShowEach() bei jedem Aufruf leise abfaengt (s. dort) -- hier
        // zusaetzlich einmalig laut beim Bringup, statt erst beim ersten Show()-Aufruf per
        // log_warn() zu bemerken, dass CubeMX die DMA-Verlinkung fuer diesen Kanal nie bekommen hat.
        uint16_t dma_id = use_update_burst_ ? TIM_DMA_ID_UPDATE : (uint16_t)((channel_ / 4U) + 1U);
        HW_CONFIG_ASSERT(htim_->hdma[dma_id] != nullptr,
                          "Kein GPDMA fuer diesen TIM-Kanal verlinkt (CubeMX: TIM15 -> DMA Settings)");

        // HAL_TIM_PWM_Start_DMA() (Show()/ShowEach()'s Nicht-Burst-Pfad) aktiviert Kanal, MOE
        // und den Timer-Zaehler automatisch mit -- HAL_TIM_DMABurst_MultiWriteStart() (Burst-
        // Pfad) tut das NICHT, verlaesst sich also stillschweigend darauf, dass irgendein
        // anderer Kanal auf demselben Timer (CH1/CH2 teilen sich TIM15's Zaehler) den Zaehler
        // bereits gestartet hat. Ohne laufenden Zaehler entstehen aber gar keine Update-Events,
        // also auch keine TIM15_UP-DMA-Requests -- ein reiner Burst-Kanal ohne begleitenden
        // Nicht-Burst-Kanal haette so nie funktioniert. Deshalb hier explizit gestartet, statt
        // sich auf die Aufrufreihenfolge in ws2812_control.cpp zu verlassen.
        if (use_update_burst_) {
            HAL_TIM_PWM_Start(htim_, channel_);
        }
    }

    // color_rgb: 0xRRGGBB, wird auf alle LedCount LEDs dieses Kanals angewendet.
    // Blockiert bis die DMA-Uebertragung angestossen ist (nicht bis sie fertig ist) --
    // Aufrufer muss vor dem naechsten Show()/ShowEach() auf demselben Kanal warten, bis die
    // vorherige Uebertragung sicher abgeschlossen ist.
    void Show(uint32_t color_rgb) {
        uint32_t colors_rgb[LedCount];
        for (uint32_t led = 0; led < LedCount; ++led) {
            colors_rgb[led] = color_rgb;
        }
        ShowEach(colors_rgb);
    }

    // Wie Show(), aber mit einer individuellen Farbe pro LED (colors_rgb[0] = erste LED in der
    // Kette usw.) -- fuer Ketten mit LedCount > 1, bei denen nicht alle LEDs dieselbe Farbe
    // zeigen sollen (z.B. abwechselnde Muster).
    void ShowEach(const uint32_t (&colors_rgb)[LedCount]) {
        if (htim_ == nullptr) {
            return;
        }

        // Beide HAL-Einstiegspunkte unten dereferenzieren ein per CubeMX/__HAL_LINKDMA
        // verlinktes DMA-Handle -- HAL_TIM_PWM_Start_DMA() ungeprueft (siehe
        // stm32h5xx_hal_tim.c, TIM_DMADelayPulseCplt-Zuweisung direkt am Funktionsanfang:
        // fehlt die Verlinkung, ist das ein Nullpointer-Zugriff -> HardFault statt
        // HAL_ERROR!). HAL_TIM_DMABurst_WriteStart() prueft zwar selbst auf NULL, liefert
        // dann aber HAL_OK zurueck, OHNE irgendetwas zu konfigurieren -- ein rueckgabewert-
        // basierter Check waere hier also ebenfalls irrefuehrend. Deshalb in beiden Faellen
        // hier selbst geprueft, statt der HAL zu vertrauen -- fehlende WS2812-Ansteuerung
        // soll nie den ganzen io_thread mitreissen.
        uint16_t dma_id = use_update_burst_ ? TIM_DMA_ID_UPDATE : (uint16_t)((channel_ / 4U) + 1U);
        if (htim_->hdma[dma_id] == nullptr) {
            log_warn("ws2812::Driver::ShowEach(): kein GPDMA fuer TIM-Kanal %lu (Request-Quelle %s) "
                     "verlinkt (CubeMX: TIM15 DMA Settings) - LED-Ausgabe uebersprungen",
                     (unsigned long)channel_, use_update_burst_ ? "Update" : "Kanal-Compare");
            return;
        }

        uint32_t idx = 0;
        for (uint32_t led = 0; led < LedCount; ++led) {
            // WS2812 erwartet GRB-Bitreihenfolge, MSB zuerst.
            const uint8_t g = (uint8_t)(colors_rgb[led] >> 8);
            const uint8_t r = (uint8_t)(colors_rgb[led] >> 16);
            const uint8_t b = (uint8_t)colors_rgb[led];
            const uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
            for (int8_t bit = 23; bit >= 0; --bit) {
                buffer_[idx++] = (grb & (1u << bit)) ? CCR_BIT1 : CCR_BIT0;
            }
        }
        buffer_[idx++] = 0; // Reset-Slot: Leitung nach dem letzten Bit auf Low

        // GPDMA (STM32H5) erwartet die Transferlaenge grundsaetzlich in BYTES (landet
        // unveraendert in CBR1.BNDT, s. stm32h5xx_hal_dma.c DMA_SetConfig() -- keine
        // Skalierung nach konfigurierter Datenbreite irgendwo im Aufrufpfad). buffer_ ist
        // uint32_t[], "idx" zaehlt aber Elemente -- ohne *4 wuerde nur ein Viertel des
        // Puffers uebertragen.
        const uint32_t length_bytes = idx * (uint32_t)sizeof(uint32_t);

        if (use_update_burst_) {
            // HAL_TIM_DMABurst_WriteStart() (der uebliche Einstiegspunkt) leitet ihre interne
            // DataLength NUR aus BurstLength (Anzahl gleichzeitig beschriebener Register) und
            // der konfigurierten DMA-Breite ab -- bei BurstLength=1TRANSFER waeren das nur 4
            // Bytes (ein Registerschreibzugriff), unabhaengig von der tatsaechlichen
            // Puffergroesse. Der gesamte Rest von buffer_ würde also nie uebertragen, das
            // Update-Event-DMA laeuft nach dem allerersten Wert einfach leer (Mode=DMA_NORMAL,
            // fertig nach einem BNDT=0). Deshalb direkt die tiefere HAL_TIM_DMABurst_
            // MultiWriteStart() mit selbst berechneter Gesamtlaenge aufgerufen, damit die GPDMA-
            // Hardware bei jedem weiteren TIM15_UP-Request automatisch den naechsten 32-Bit-Wert
            // aus buffer_ nachlaedt, bis alle idx Werte durch sind.
            uint32_t burst_base = TIM_DMABASE_CCR1 + (channel_ / 4U);
            HAL_TIM_DMABurst_WriteStop(htim_, TIM_DMA_UPDATE);
            HAL_TIM_DMABurst_MultiWriteStart(htim_, burst_base, TIM_DMA_UPDATE, buffer_,
                                              TIM_DMABURSTLENGTH_1TRANSFER, length_bytes);
        } else {
            HAL_TIM_PWM_Stop_DMA(htim_, channel_);
            HAL_TIM_PWM_Start_DMA(htim_, channel_, buffer_, (uint16_t)length_bytes);
        }
    }

private:
    // MX_TIM15_Init() (main.c, CubeMX-generiert) konfiguriert CH1/CH2 im reinen Output-Compare-
    // "Timing"-Modus (kein Ausgang) -- fuer PWM-per-DMA muss das auf echten PWM-Modus 1
    // umgestellt werden, gleiches Muster wie bei TIM4 CH3/CH4 in io_thread.cpp (dort auch die
    // Begruendung, warum das hier statt in main.c passiert). Noetig unabhaengig von der
    // DMA-Strategie: die DMA-Burst-Adressierung fuellt nur den CCRx-Registerwert, die
    // eigentliche PWM-Wellenform (Vergleich CNT vs. CCRx, Pin-Ausgabe) braucht trotzdem den
    // normalen PWM1-Modus auf diesem Kanal.
    static void configure_pwm_channel(TIM_HandleTypeDef *htim, uint32_t channel) {
        TIM_OC_InitTypeDef sConfigOC = {0};
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = 0;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel);
    }

    TIM_HandleTypeDef *htim_ = nullptr;
    uint32_t channel_ = 0;
    bool use_update_burst_ = false;
    // +1 Reset-Slot: haelt die Leitung nach dem letzten Bit auf Low (CCR=0), bis Stop_DMA()
    // aufgerufen wird -- WS2812 braucht danach ohnehin >50us Low fuer den Reset/Latch, den der
    // Aufrufer per tx_thread_sleep() zwischen zwei Show()-Aufrufen sicherstellt.
    uint32_t buffer_[LedCount * 24 + 1];
};

} // namespace ws2812
