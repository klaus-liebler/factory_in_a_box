#include "greeting.hpp"
#include "log.h"
#include "gitconstants.hh"
#include "generated/device_certificate.h"
#include "main.h"

static constexpr std::string_view BOARD_NAME = "Factory Control Unit (STM32H563)";

extern "C" ETH_HandleTypeDef heth;

// Grund des letzten Resets (RCC->RSR) -- hilft bei der Ferndiagnose deutlich mehr als ein
// blosses "Geraet ist neu gebootet": unterscheidet z.B. einen unerwarteten Watchdog-Reset
// (Firmware haengt irgendwo) von einem normalen Power-On oder einem gewollten Software-Reset.
// Absichtlich NICHT __HAL_RCC_CLEAR_RESET_FLAGS() aufgerufen -- die Flags bleiben fuer andere
// Stellen/Debugger sichtbar, bis der naechste Reset sie ohnehin ueberschreibt.
static const char *reset_cause() {
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) return "Independent Watchdog";
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) return "Window Watchdog";
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))  return "Software";
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) return "Low-Power";
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))  return "Brown-Out";
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))  return "Reset Pin / Power-On";
    return "Unknown";
}

void greeting() {
    log_info("");
    log_info("=== FactoryInABox Control Unit starting ===");
    log_info("Board:    %.*s", (int)BOARD_NAME.length(), BOARD_NAME.data());
    log_info("Hostname: %s.local", DEVICE_HOSTNAME);
    log_info("--- Build Information ---");
    log_info("  Version: %.*s", (int)git::VERSION.length(), git::VERSION.data());
    log_info("  Commit:  %.*s (%.*s)", (int)git::COMMIT_HASH.length(), git::COMMIT_HASH.data(),
              (int)git::BRANCH.length(), git::BRANCH.data());
    log_info("  Message: %.*s", (int)git::COMMIT_MESSAGE.length(), git::COMMIT_MESSAGE.data());
    log_info("  Author:  %.*s", (int)git::COMMIT_AUTHOR.length(), git::COMMIT_AUTHOR.data());
    log_info("  Built:   %.*s", (int)git::BUILD_TIMESTAMP.length(), git::BUILD_TIMESTAMP.data());
    if (git::IS_DIRTY) {
        log_warn("Working directory has uncommitted changes!");
    } else {
        log_info("Working directory was clean relative to Git.");
    }

    log_info("--- Clocks ---");
    log_info("  SYSCLK: %lu MHz", (unsigned long)(HAL_RCC_GetSysClockFreq() / 1000000));
    log_info("  HCLK:   %lu MHz", (unsigned long)(HAL_RCC_GetHCLKFreq() / 1000000));
    log_info("  PCLK1:  %lu MHz", (unsigned long)(HAL_RCC_GetPCLK1Freq() / 1000000));
    log_info("  PCLK2:  %lu MHz", (unsigned long)(HAL_RCC_GetPCLK2Freq() / 1000000));

    // UID_BASE liegt im OTP-/System-Speicherbereich, nicht im regulaeren Flash-Adressraum, den
    // der ICACHE ueberwacht -- ein Read bei aktivem ICACHE erzeugt einen precise BusFault
    // (gleiches Muster wie in diagnostics.hpp).
    HAL_ICACHE_Disable();
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();
    HAL_ICACHE_Enable();
    log_info("--- Chip ---");
    log_info("  UID: %08lX-%08lX-%08lX", (unsigned long)uid0, (unsigned long)uid1, (unsigned long)uid2);
    log_info("  Reset cause: %s", reset_cause());

    // MACAddr wird in main.c (MX_ETH_Init) aktuell fuer alle Boards identisch hartcodiert
    // (00:80:E1:00:00:00) -- kein zufaellig/eindeutig generierter Wert. Bei mehreren Geraeten
    // im selben Netz fuehrt das zu einer MAC-Adresskollision (ARP-Chaos, nicht nur kosmetisch).
    // Hier trotzdem mit ausgegeben, weil das genau der Wert ist, unter dem das Board tatsaechlich
    // im Netz sichtbar ist -- die Kollisionsgefahr ist ein eigenes, noch offenes Thema.
    log_info("  MAC: %02X:%02X:%02X:%02X:%02X:%02X",
              heth.Init.MACAddr[0], heth.Init.MACAddr[1], heth.Init.MACAddr[2],
              heth.Init.MACAddr[3], heth.Init.MACAddr[4], heth.Init.MACAddr[5]);

    log_info("============================================");
}
