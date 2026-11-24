// Login mode per interface (src/auth_mode.hpp, docs/security.md "Login mode per interface"): what a
// request without login may do over an open Ethernet or an open USB-NCM, with the FCU's role table.
#include <gtest/gtest.h>

#include <optional>

#include "auth_mode.hpp"
#include "ws_permissions.hpp"

namespace {

namespace w = WsProtocol;
using fcu::auth::InterfaceModes;
using fcu::auth::LoginMode;
using fcu::auth::open_role;
using fcu::auth::ws_permission;
using rtos::auth::Decision;
using rtos::auth::Role;
using rtos::auth::Subject;
using rtos::auth::decide;
using rtos::auth::effective_subject;

/// The subject of a request without login, as auth.cpp forms it (presence: the USB interface).
Subject anonymous(const InterfaceModes& m, bool usb) { return effective_subject(std::nullopt, m.of(usb), open_role, usb); }

const auto read = ws_permission(w::modbus::NAMESPACE_ID, w::modbus::GetRegistersRequest::TYPE_ID);
const auto write = ws_permission(w::modbus::NAMESPACE_ID, w::modbus::WriteHoldingRequest::TYPE_ID);
const auto download = ws_permission(w::plc::NAMESPACE_ID, w::plc::UploadRequest::TYPE_ID);
const auto terminal = ws_permission(w::terminal::NAMESPACE_ID, w::terminal::CommandRequest::TYPE_ID);

TEST(AuthMode, DefaultNeedsALoginOnEveryInterface) {
    constexpr InterfaceModes m{};
    EXPECT_FALSE(m.open(false));
    EXPECT_FALSE(m.open(true));
    EXPECT_EQ(decide(anonymous(m, false), read), Decision::NotAuthenticated);
    EXPECT_EQ(decide(anonymous(m, true), read), Decision::NotAuthenticated);
}

TEST(AuthMode, OpenUsbIsProgrammerWithPresenceButNoAdmin) {
    constexpr InterfaceModes m{.ethernet = LoginMode::Login, .usb = LoginMode::Open};
    EXPECT_EQ(decide(anonymous(m, true), write), Decision::Allowed);
    EXPECT_EQ(decide(anonymous(m, true), download), Decision::Allowed);  // USB-NCM is presence
    EXPECT_EQ(decide(anonymous(m, true), terminal), Decision::RoleTooLow);
    EXPECT_EQ(decide(anonymous(m, false), read), Decision::NotAuthenticated);  // Ethernet stays closed
}

TEST(AuthMode, OpenEthernetCannotDownloadWithoutPresence) {
    constexpr InterfaceModes m{.ethernet = LoginMode::Open, .usb = LoginMode::Login};
    EXPECT_EQ(decide(anonymous(m, false), write), Decision::Allowed);
    EXPECT_EQ(decide(anonymous(m, false), download), Decision::PresenceRequired);
    EXPECT_EQ(decide(anonymous(m, false), terminal), Decision::RoleTooLow);
    // an admin who logs in on the open interface gets the admin role
    const Subject admin{.role = Role::Admin};
    EXPECT_EQ(decide(effective_subject(admin, m.of(false), open_role, false), terminal), Decision::Allowed);
}

}  // namespace
