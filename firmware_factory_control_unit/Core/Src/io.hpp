#pragma once
#include "tx_api.h"
#include "nx_api.h"
#include "nxd_dhcp_client.h"
#include "modbus_commons.hh"
#include "single_led.hh"
#include "usb_pd_control.hpp"

#include "setup_and_loops/interfaces.hh"
#include "setup_and_loops/can.hh"
#include "setup_and_loops/eth_link.hh"
#include "setup_and_loops/scale.hh"
#include "setup_and_loops/stepper.hh"
#include "setup_and_loops/tof_color.hh"
#include "setup_and_loops/ws2812.hh"

// 5 Ticks * 10ms/Tick (TX_TIMER_TICKS_PER_SECOND=100, s. tx_user.h) = 50ms.
constexpr ULONG IO_THREAD_SLEEP_TICKS = 5;

class Io {
    private:
    Modbus::IModbusRegisterModel& register_model;
    NX_IP& ip_instance;
    NX_DHCP& dhcp_client;
    USBPDControl& usb_pd_control;

    single_led::M<false> info_led;
    single_led::BlinkPattern heartbeat_pattern_;

    CanSetupAndLoop can_setup_and_loop;
    EthLink eth_link;
    ScaleSetupAndLoop scale_;
    // Referenz statt eigenem Member: App legt StepperSetupAndLoop frueh an (SetupBeforeThreadX(),
    // vor tx_kernel_enter()) und ruft SetupEarly() selbst auf -- s. Klassenkommentar in
    // setup_and_loops/stepper.hh. Io ruft hier nur noch Setup() (No-Op)/Loop() auf.
    StepperSetupAndLoop& stepper_;
    TofColorSetupAndLoop tof_color_;
    Ws2812SetupAndLoop ws2812_;

    void readInputs(uint32_t now);
    void processMembers(uint32_t now);
    void updateOutputs(uint32_t now);

    public:
    Io(Modbus::IModbusRegisterModel& register_model, NX_IP& ip_instance, NX_DHCP& dhcp_client,
       USBPDControl& usb_pd_control, StepperSetupAndLoop& stepper)
        : register_model(register_model), ip_instance(ip_instance), dhcp_client(dhcp_client), usb_pd_control(usb_pd_control),
          info_led(gpio::Pin::PE15),
          heartbeat_pattern_(50, 50),
          can_setup_and_loop(register_model),
          eth_link(register_model, ip_instance, dhcp_client),
          scale_(register_model),
          stepper_(stepper),
          tof_color_(register_model),
          ws2812_(register_model) {}

    void Setup();
    void Loop();
};
