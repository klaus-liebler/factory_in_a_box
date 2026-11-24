// Web UI authentication of the FCU (docs/security.md): accounts and roles, login sessions, the
// physical-presence rule and the audit trail; rtos::auth provides the building blocks.
//
//   Accounts   the provisioning account "admin" with the per-device one-time password from the
//              builder (generated/device_credentials.hh: salt + PBKDF2 hash only; the password is
//              on the label and in the board archive). It must be changed at the first login.
//              Further accounts are created by an admin. The table lives in two flash sectors
//              below the PLC slots (CredentialStore, A/B); the log thread writes them (it may block,
//              the network thread may not).
//   Sessions   POST /api/login (JSON user/password) checks the password in background steps of the
//              network thread, then sets the HttpOnly cookie fcu_session (Secure, SameSite=Strict).
//              Inactivity 30 min, lifetime 12 h, POST /api/logout. The WebSocket upgrade needs a
//              valid session (and the Origin check of the server); every WebSocket message is
//              checked against the role table (ws_permissions.hpp) on the device.
//   Throttle   per client address and globally (rtos/auth/throttle.hpp): answered with 429 at once.
//   Presence   PLC download, forcing, firmware/certificate changes and password resets need physical
//              presence: the session came in over the USB interface (a cable at the device), or a
//              presence button was pressed in the last two minutes - no FCU revision has such a
//              button yet (src/hal/rev*/pins.hpp), so today it is USB; presence_pressed() is the hook.
//   Recovery   POST /api/recover over USB without login: restores the label password of "admin"
//              (must change again) and ends every session.
//   Audit      every login, failure, lockout, logout, password and role change, and every refused or
//              critical WebSocket request goes into the log journal ("auth:" lines).
#pragma once

#include <cstdint>

#include "network.hpp"
#include "rtos/auth/access.hpp"

namespace fcu::auth {

using SessionId = net::Server::SessionId;
using rtos::auth::Decision;
using rtos::auth::Permission;
using rtos::auth::Role;

/// WebSocket error codes of refused requests (WsProtocol::ErrorCode::Application + n); the web UI
/// knows them (web/src/core/auth.ts).
inline constexpr std::uint16_t error_not_authenticated = 0x101;
inline constexpr std::uint16_t error_role = 0x102;
inline constexpr std::uint16_t error_presence = 0x103;
inline constexpr std::uint16_t error_password_change = 0x104;

/// Loads the accounts (flash, else the provisioning account). Network thread, before the server starts.
void init() noexcept;

/// The /api/ handler and the WebSocket upgrade check for the HTTPS server.
[[nodiscard]] const net::Server::ApiHandler& api_handler() noexcept;

/// May WebSocket session `id` do what `need` requires? Refreshes the login's inactivity timer; a
/// refused request is logged. Network thread.
[[nodiscard]] Decision authorize(SessionId id, Permission need, const char* what) noexcept;

/// The WebSocket of `id` closed.
void websocket_closed(SessionId id) noexcept;

/// Role of the login behind WebSocket `id` (Role::None: none).
[[nodiscard]] Role role_of(SessionId id) noexcept;

/// Network-thread service: ends WebSockets whose login expired, ended or was replaced.
struct Service {
    void service() noexcept;
    [[nodiscard]] std::uint32_t next_deadline() const noexcept;
};
Service& service() noexcept;

/// Log thread: writes the account table if it changed (blocks while the flash erases).
void store_pending_changes() noexcept;

/// A presence button was pressed (any thread). No revision has one yet.
void presence_pressed() noexcept;

}  // namespace fcu::auth
