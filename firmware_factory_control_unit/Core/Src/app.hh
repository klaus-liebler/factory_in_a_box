// ============================================================================
// ThreadX Application Wiring -- erzeugt alle Pools/Threads (tx_application_define(), laeuft
// vor dem Scheduler) und orchestriert danach den Boot-Ablauf (App::AppThread(), der einzige
// Thread mit TX_AUTO_START). Nur drei ThreadX-Threads insgesamt:
//   - app_main_thread: einmaliger Boot-Orchestrator (FileX/TLS/HTTP/DHCP/Modbus-Setup, dann
//     Start der beiden anderen Threads)
//   - modbus_server_thread: blockierender TCP-Accept-Loop (ModbusTcpServer::run())
//   - io_thread: EIN gemeinsamer Thread fuer alle Sensoren/Aktoren (s. io.cpp fuer die
//     Begruendung, warum das zusammengefasst werden konnte)
// ============================================================================
#pragma once
#include <cstdint>

#include "net_setup.hpp"
#include "io.hpp"
#include "usb_pd_control.hpp"

#include "modbus_register_model.hh"
#include "modbus_tcp_server.hpp"
#include "log.h"

#include "nx_api.h"
#include "fx_api.h"
#include "tx_api.h"
#include "nxd_dhcp_client.h"
#include "nx_web_http_server.h"
#include "nxd_mdns.h"
#include "common_macros.hh"
#include "constants.hh"
#include "main.h"

extern "C" ETH_HandleTypeDef heth;


class App {
private:
    App() = default;
    ~App() = default;
    const char *reset_cause() {
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) return "Independent Watchdog";
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) return "Window Watchdog";
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))  return "Software";
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) return "Low-Power";
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))  return "Brown-Out";
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))  return "Reset Pin / Power-On";
        return "Unknown";
    }
    void greeting();
    void fillRegistersWithInitialValues();
public:
    static App& Instance() {
        static App app_instance;
        return app_instance;
    }

    TX_BYTE_POOL byte_pool;
    NX_PACKET_POOL packet_pool;

    NX_IP ip_instance;
    NX_DHCP dhcp_client;
    NX_WEB_HTTP_SERVER http_server;
    NX_MDNS mdns;
    ModbusTcpServer* modbus_server = nullptr;
    Io* io = nullptr;
    USBPDControl* usb_pd_control = nullptr;

    FX_MEDIA sd_media;

    ULONG ip_address;
    ULONG net_mask;

    TX_THREAD app_main_thread;
    TX_THREAD modbus_server_thread;
    TX_THREAD io_thread;
    uint32_t chip_uid[3];
    ModbusRegisterModel* register_model = nullptr;

    void SetupBeforeThreadX();
    void AppThread();
    void ModbusServerThread();
    void IOThread();
    static void AppThreadStatic(ULONG arg);
    static void ModbusServerThreadStatic(ULONG arg);
    static void IOThreadStatic(ULONG arg);
};
