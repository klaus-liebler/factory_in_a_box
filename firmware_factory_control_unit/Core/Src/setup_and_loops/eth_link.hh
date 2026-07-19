#pragma once
// ============================================================================
// Ethernet-Link-Ueberwachung -- Aufruf einmal pro io_thread-Zyklus (s. io.cpp).
// ============================================================================
#include "interfaces.hh"
#include "modbus_register_model.hh"
#include "log.h"
#include "nx_stm32_phy_driver.h"
#include "nxd_dhcp_client.h"

class EthLink : public ISetupAndLoop {
    private:
    Modbus::IModbusRegisterModel& register_model;
    NX_IP& ip_instance;
    NX_DHCP& dhcp_client;

    void update_eth_link_registers(bool link_up) {
    uint16_t speed = 0;   // Mbit/s (0 = kein Link; LAN8742 kann nur 10/100, kein Gigabit)
    uint16_t duplex = 0;  // 0=half, 1=full

    if (link_up) {
        switch (nx_eth_phy_get_link_state()) {
            case ETH_PHY_STATUS_100MBITS_FULLDUPLEX:
                speed = 100; duplex = 1; break;
            case ETH_PHY_STATUS_100MBITS_HALFDUPLEX:
                speed = 100; duplex = 0; break;
            case ETH_PHY_STATUS_10MBITS_FULLDUPLEX:
                speed = 10; duplex = 1; break;
            case ETH_PHY_STATUS_10MBITS_HALFDUPLEX:
            default:
                speed = 10; duplex = 0; break;
        }
    }

    register_model.SetInputRegister(ModbusRegisters::Input::ETH_LINK_STATUS, link_up ? 1 : 0);
    register_model.SetInputRegister(ModbusRegisters::Input::ETH_LINK_SPEED, speed);
    register_model.SetInputRegister(ModbusRegisters::Input::ETH_LINK_DUPLEX, duplex);
}

    public:
    EthLink(Modbus::IModbusRegisterModel& model, NX_IP& ip_instance, NX_DHCP& dhcp_client)
        : register_model(model), ip_instance(ip_instance), dhcp_client(dhcp_client) {}

    void Setup() override {
        // Kein echter PHY-Interrupt verdrahtet/konfiguriert -- der Link-Status wird stattdessen
        // jeden Loop()-Zyklus per nx_ip_interface_status_check() gepollt (s. Loop() unten).
    }

    void Loop(uint32_t now) override {
        (void)now;
        // -1 = Zustand beim Boot noch unbekannt -- erzwingt beim ersten Aufruf einen Durchlauf
        // des passenden Zweigs (verbunden oder getrennt), egal ob das Kabel beim Start bereits
        // steckt oder nicht. Mit Startwert 0 ("nehmen Verbindung an") wuerde der Up-Zweig nie
        // feuern, wenn das Kabel schon beim Boot steckt, und update_eth_link_registers(true)
        // bliebe aus. static, da der Zustand zwischen einzelnen Aufrufen erhalten bleiben muss.
        static int linkdown = -1;
        // Getrennt von linkdown gemerkt: der allererste Loop()-Aufruf (Kabel war beim Boot
        // schon gesteckt) darf NICHT wie ein echtes Reconnect behandelt werden -- DHCP lief zu
        // diesem Zeitpunkt bereits erfolgreich in net_setup_start(), ein erneuter
        // nx_dhcp_stop()/start() wuerde nur eine redundante zweite IP-Adressaenderung (und damit
        // eine doppelte Log-Zeile in ip_address_change_notify_callback()) erzeugen. Bei einem
        // echten Reconnect (Kabel gezogen und wieder gesteckt) ist der DHCP-Neustart dagegen
        // noetig, falls sich die Adresse/der Lease inzwischen geaendert hat.
        bool is_first_call = (linkdown == -1);

        ULONG actual_status;
        UINT status = nx_ip_interface_status_check(&ip_instance, 0,
                                                    NX_IP_LINK_ENABLED,
                                                    &actual_status, 10);

        if (status == NX_SUCCESS) {
            if (linkdown != 0) {
                linkdown = 0;
                log_info("Network cable connected");
                nx_ip_driver_direct_command(&ip_instance, NX_LINK_ENABLE, &actual_status);

                if (!is_first_call) {
                    status = nx_ip_interface_status_check(&ip_instance, 0,
                                                          NX_IP_ADDRESS_RESOLVED,
                                                          &actual_status, 10);
                    if (status == NX_SUCCESS) {
                        nx_dhcp_stop(&dhcp_client);
                        nx_dhcp_reinitialize(&dhcp_client);
                        nx_dhcp_start(&dhcp_client);
                    }
                }

                update_eth_link_registers(true);
            }
        } else {
            if (linkdown != 1) {
                linkdown = 1;
                log_info("Network cable disconnected");
                nx_ip_driver_direct_command(&ip_instance, NX_LINK_DISABLE, &actual_status);

                update_eth_link_registers(false);
            }
        }
    }
};
