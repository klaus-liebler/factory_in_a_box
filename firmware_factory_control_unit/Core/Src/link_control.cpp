// ============================================================================
// Ethernet-Link-Ueberwachung -- Aufruf einmal pro io_thread-Zyklus (s. io_thread.cpp).
// ============================================================================
#include "link_control.hpp"
#include "app_state.hpp"
#include "modbus_register_map.hpp"
#include "log.h"
#include "nx_stm32_phy_driver.h"

// Schreibt Link-Status + (bei Link-Up) die von der LAN8742-PHY tatsächlich
// ausgehandelte Speed/Duplex ins Registermodell. Wird nur bei einer erkannten
// Verbindungsänderung aufgerufen (siehe link_status_update), nicht periodisch --
// die Werte aendern sich nur bei einer neuen Auto-Negotiation.
static void update_eth_link_registers(bool link_up) {
    // g_app_state.modbus_server ist hier immer gesetzt: link_status_update() wird erst
    // aus dem io_thread heraus aufgerufen, lange nachdem modbus_server in
    // tx_application_define() zugewiesen wurde.
    ModbusTcpServer& server = *g_app_state.modbus_server;

    uint16_t speed = 0;   // 0=10M, 1=100M, 2=1000M
    uint16_t duplex = 0;  // 0=half, 1=full

    if (link_up) {
        switch (nx_eth_phy_get_link_state()) {
            case ETH_PHY_STATUS_100MBITS_FULLDUPLEX:
                speed = 1; duplex = 1; break;
            case ETH_PHY_STATUS_100MBITS_HALFDUPLEX:
                speed = 1; duplex = 0; break;
            case ETH_PHY_STATUS_10MBITS_FULLDUPLEX:
                speed = 0; duplex = 1; break;
            case ETH_PHY_STATUS_10MBITS_HALFDUPLEX:
            default:
                speed = 0; duplex = 0; break;
        }
    }

    server.write_input_register(ModbusRegisters::Input::ETH_LINK_STATUS, link_up ? 1 : 0);
    server.write_input_register(ModbusRegisters::Input::ETH_LINK_SPEED, speed);
    server.write_input_register(ModbusRegisters::Input::ETH_LINK_DUPLEX, duplex);
}

void link_status_update() {
    // -1 = Zustand beim Boot noch unbekannt -- erzwingt beim ersten Aufruf einen Durchlauf
    // des passenden Zweigs (verbunden oder getrennt), egal ob das Kabel beim Start bereits
    // steckt oder nicht. Mit Startwert 0 ("nehmen Verbindung an") wuerde der Up-Zweig nie
    // feuern, wenn das Kabel schon beim Boot steckt, und update_eth_link_registers(true)
    // bliebe aus. static, da der Zustand jetzt zwischen einzelnen Aufrufen (statt in einer
    // eigenen while(1)-Schleife) erhalten bleiben muss.
    static int linkdown = -1;

    ULONG actual_status;
    UINT status = nx_ip_interface_status_check(&g_app_state.ip_instance, 0,
                                                NX_IP_LINK_ENABLED,
                                                &actual_status, 10);

    if (status == NX_SUCCESS) {
        if (linkdown != 0) {
            linkdown = 0;
            log_info("Network cable connected");
            nx_ip_driver_direct_command(&g_app_state.ip_instance, NX_LINK_ENABLE, &actual_status);

            status = nx_ip_interface_status_check(&g_app_state.ip_instance, 0,
                                                  NX_IP_ADDRESS_RESOLVED,
                                                  &actual_status, 10);
            if (status == NX_SUCCESS) {
                nx_dhcp_stop(&g_app_state.dhcp_client);
                nx_dhcp_reinitialize(&g_app_state.dhcp_client);
                nx_dhcp_start(&g_app_state.dhcp_client);
            }

            update_eth_link_registers(true);
        }
    } else {
        if (linkdown != 1) {
            linkdown = 1;
            log_info("Network cable disconnected");
            nx_ip_driver_direct_command(&g_app_state.ip_instance, NX_LINK_DISABLE, &actual_status);

            update_eth_link_registers(false);
        }
    }
}
