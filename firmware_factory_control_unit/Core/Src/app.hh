// ============================================================================
// ThreadX Application Wiring -- erzeugt alle Pools/Threads (tx_application_define(), laeuft
// vor dem Scheduler) und orchestriert danach den Boot-Ablauf (App::AppThread(), der einzige
// Thread mit TX_AUTO_START). Vier ThreadX-Threads insgesamt (bewusst nicht mehr nur drei, s.
// usbd_device_thread unten):
//   - app_main_thread: einmaliger Boot-Orchestrator (FileX/TLS/HTTP/DHCP/Modbus-Setup, dann
//     Start der uebrigen Threads)
//   - modbus_server_thread: blockierender TCP-Accept-Loop (ModbusTcpServer::run())
//   - io_thread: EIN gemeinsamer Thread fuer alle Sensoren/Aktoren (s. io.cpp fuer die
//     Begruendung, warum das zusammengefasst werden konnte)
//   - usbd_device_thread: USBX-Lebenszyklus-Schleife (usbd_device_loop(), s. usbd_device.c) --
//     ueberwacht nur noch USB_VSENSE und stoesst ModbusRtuServer::Poll() an; die eigentliche
//     USB-Bedienung (Enumeration/Bulk-Transfers) laeuft seit dem Umstieg von TinyUSB auf USBX
//     in klasseneigenen ThreadX-Threads (s. Klassenkommentar in usbd_device.h), nicht mehr hier.
//   - heartbeat_thread: reine Diagnose (Log-Zeile alle 3s: CPU-Temperatur/freier Heap/
//     Bus-Spannung/Strom). Bewusst NICHT im io_thread (koennte durch dessen Sensor-/Aktor-
//     Zyklus verzoegert werden) -- liest nur bereits von io_thread periodisch aktualisierte
//     Register (kein eigener I2C/DTS-Zugriff, keine Konkurrenz zu deren Peripherie-Besitz).
// ============================================================================
#pragma once
#include <cstdint>

#include "net_setup.hpp"
#include "io.hpp"
#include "usb_pd_control.hpp"
#include "usbd_device.h"
#include "hal_tick_threadx.h"

#include "modbus_register_model.hh"
#include "modbus_tcp_server.hpp"
#include "modbus_rtu_server.hpp"
#include "log.h"

#include "nx_api.h"
#include "fx_api.h"
#include "tx_api.h"
#include "nxd_dhcp_client.h"
#include "nxd_dhcp_server.h"
#include "nx_web_http_server.h"
#include "nxd_mdns.h"
#include "common_macros.hh"
#include "constants.hh"
#include "main.h"

extern "C" ETH_HandleTypeDef heth;


// Werte von App::ResetCauseCode() -- ueber /api/system nach aussen gereicht (s. webserver.cpp),
// da die zugrundeliegenden RCC-Reset-Flags direkt nach dem Auslesen in greeting() geloescht
// werden (__HAL_RCC_CLEAR_RESET_FLAGS()) und danach nicht mehr abfragbar waeren.
enum class ResetCause : uint8_t {
    Unknown = 0,
    IndependentWatchdog = 1,
    WindowWatchdog = 2,
    Software = 3,
    LowPower = 4,
    BrownOut = 5,
    PinOrPowerOn = 6,
};

class App {
private:
    App() = default;
    ~App() = default;
    // Liest die RCC-Reset-Flags aus (nur gueltig VOR __HAL_RCC_CLEAR_RESET_FLAGS() in greeting()),
    // merkt sich den Code in reset_cause_code_ (fuer /api/system, s. ResetCauseCode()) und gibt
    // zugleich den Klartext fuers Boot-Log zurueck.
    const char *reset_cause() {
        ResetCause code = ResetCause::Unknown;
        const char *text = "Unknown";
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) { code = ResetCause::IndependentWatchdog; text = "Independent Watchdog"; }
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) { code = ResetCause::WindowWatchdog; text = "Window Watchdog"; }
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))  { code = ResetCause::Software; text = "Software"; }
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) { code = ResetCause::LowPower; text = "Low-Power"; }
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))  { code = ResetCause::BrownOut; text = "Brown-Out"; }
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))  { code = ResetCause::PinOrPowerOn; text = "Reset Pin / Power-On"; }
        reset_cause_code_ = static_cast<uint8_t>(code);
        return text;
    }
    void greeting();
    void fillRegistersWithInitialValues();
    uint8_t reset_cause_code_ = 0;
public:
    static App& Instance() {
        static App app_instance;
        return app_instance;
    }

    uint8_t ResetCauseCode() const { return reset_cause_code_; }

    TX_BYTE_POOL byte_pool;
    NX_PACKET_POOL packet_pool;

    NX_IP ip_instance;
    NX_DHCP dhcp_client;
    NX_WEB_HTTP_SERVER http_server;
    // Zusaetzlich auf dem virtuellen USB-Netzwerk-Interface aktiviert (s. net_setup.cpp) -- eine
    // zweite NX_MDNS-Instanz fuer einen eigenen festen Namen scheiterte an NX_PORT_UNAVAILABLE
    // (beide haetten denselben UDP-Port 5353 auf derselben NX_IP gebunden), daher weiterhin nur
    // diese eine Instanz, jetzt auf beiden Interfaces.
    NX_MDNS mdns;
    // DHCP-Server fuer die virtuelle USB-CDC-NCM-NIC (vergibt 192.168.173.2 an den per USB
    // verbundenen Host) -- Gegenstueck zu dhcp_client oben, der stattdessen als DHCP-Client auf
    // dem Ethernet-Interface laeuft.
    NX_DHCP_SERVER dhcp_server;
    ModbusTcpServer* modbus_server = nullptr;
    ModbusRtuServer* modbus_rtu_server = nullptr;
    Io* io = nullptr;
    USBPDControl* usb_pd_control = nullptr;

    FX_MEDIA sd_media;

    ULONG ip_address;
    ULONG net_mask;

    TX_THREAD app_main_thread;
    TX_THREAD modbus_server_thread;
    TX_THREAD io_thread;
    TX_THREAD usbd_device_thread;
    TX_THREAD heartbeat_thread;
    uint32_t chip_uid[3];
    ModbusRegisterModel* register_model = nullptr;

    void SetupBeforeThreadX();
    void AppThread();
    void ModbusServerThread();
    void IOThread();
    [[noreturn]] void UsbdDeviceThread();
    [[noreturn]] void HeartbeatThread();
    static void AppThreadStatic(ULONG arg);
    static void ModbusServerThreadStatic(ULONG arg);
    static void IOThreadStatic(ULONG arg);
    static void UsbdDeviceThreadStatic(ULONG arg);
    static void HeartbeatThreadStatic(ULONG arg);
    // Wird von usbd_device_loop() (reines C, s. usbd_device.h) in jedem Schleifendurchlauf
    // aufgerufen -- einziger Weg, aus der C-Schleife heraus regelmaessig C++-Code (hier:
    // ModbusRtuServer::Poll()) anzustossen, ohne usbd_device.c C++-Wissen beizubringen (s.
    // dortiger Klassenkommentar zum Grund fuer die strikte C/C++-Trennung). Die CDC-NCM-Klasse
    // (s. usbd_ncm.c) braucht dagegen KEIN Polling mehr -- sie haengt ueber USBX' generischen
    // ux_network_driver rein ereignisgetrieben an NetX Duo, exakt wie USBX' eigene CDC-ECM/
    // RNDIS-Klassen.
    static void UsbdDevicePollHook();

    // Locally-administered, aus der Board-UID abgeleitete MAC-Adresse fuer die virtuelle
    // CDC-NCM-NIC (Byte 0 = 0x02, s. IEEE 802-2014 Tabelle 8-1 "locally administered unicast").
    // An genau dieser einen Stelle berechnet und sowohl an usbd_device_setup() (MAC-Adress-
    // String-Deskriptor der Klassenregistrierung) als auch an net_setup.cpp's
    // nx_ip_interface_attach()-Aufruf (physische Interface-Adresse) weitergereicht (aus zwei
    // verschiedenen Uebersetzungseinheiten, s. app.cc/net_setup.cpp), damit garantiert beide
    // dieselbe Adresse verwenden.
    static void ComputeEcmMac(const uint32_t chip_uid[3], uint8_t mac[6]);
};
