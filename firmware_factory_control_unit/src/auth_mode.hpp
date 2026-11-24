// Login mode per network interface (WebAuth:Modes in builder/appsettings.json, docs/security.md
// "Login mode per interface"): Ethernet and USB-NCM each either need a login (the one-time password
// of the label, then the accounts; the default) or are open. Whoever reaches the device over an open
// interface works as Programmer without logging in: values, actuators, PLC programming. Accounts and
// the terminal stay admin-only and need a login. Open replaces the login, never the presence rule:
// presence still comes from the USB interface (or the button). Header only (tests/host/auth_mode_test.cpp).
#pragma once

#include <cstdint>

#include "rtos/auth/access.hpp"

namespace fcu::auth {

using rtos::auth::LoginMode;

/// The role of a request without login on an open interface.
inline constexpr rtos::auth::Role open_role = rtos::auth::Role::Programmer;

struct InterfaceModes {
    LoginMode ethernet = LoginMode::Login;
    LoginMode usb = LoginMode::Login;

    /// The mode of the interface a request came in on (usb: the USB CDC-NCM interface).
    [[nodiscard]] constexpr LoginMode of(bool usb_interface) const noexcept { return usb_interface ? usb : ethernet; }
    [[nodiscard]] constexpr bool open(bool usb_interface) const noexcept { return of(usb_interface) == LoginMode::Open; }
};

}  // namespace fcu::auth
