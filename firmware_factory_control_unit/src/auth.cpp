// Web UI authentication of the FCU (see auth.hpp).
#include "auth.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string_view>

#include "board/chip.hpp"
#include "board/fcu.hpp"
#include "device_credentials.hh"
#include "mcu.h"  // the revision's MCU: its flash_program.hpp
#include "rtos/auth/credentials.hpp"
#include "rtos/auth/session.hpp"
#include "rtos/auth/throttle.hpp"
#include "rtos/time.hpp"
#include "system.hpp"

namespace fcu::auth {
namespace {

namespace ra = rtos::auth;
using Server = net::Server;
using ApiRequest = Server::ApiRequest;
using ApiResponse = Server::ApiResponse;
using ApiResult = Server::ApiResult;

// ---- configuration ------------------------------------------------------------------------------------
constexpr std::size_t max_users = 8;
constexpr std::size_t max_logins = 6;  // sessions (browsers, tabs share one)
/// PBKDF2 iterations per background step: about 0.5-1 ms of CPU (docs/security.md "Login cost").
constexpr std::uint32_t iterations_per_step = 32;
constexpr std::string_view cookie_name = "fcu_session";
constexpr std::uint32_t check_period_ms = 1000;

// ---- accounts and their flash -------------------------------------------------------------------------
// Two sectors right below the PLC's program slots (0x081C0000, plc.cpp); FLASH_DATA_START in
// CMakeLists.txt keeps the image out of both.
constexpr std::uintptr_t slots_base = 0x081C'0000;
constexpr std::uintptr_t store_base = slots_base - 2 * board::chip::flash::sector_bytes;
using StoreFlash = board::chip::flash::Region<store_base, 2 * board::chip::flash::sector_bytes, board::sysclk_hz>;
StoreFlash store_flash;
ra::CredentialStore<StoreFlash, max_users> store{store_flash};
using Users = ra::UserTable<max_users>;

/// The provisioning: its account and its epoch (a new label password replaces the stored table).
constexpr std::uint32_t provisioning_epoch = [] {
    std::uint32_t e = 0x811C9DC5u;  // FNV-1a over salt and hash
    for (const std::uint8_t b : DEVICE_WEB_PASSWORD_SALT) e = (e ^ b) * 0x01000193u;
    for (const std::uint8_t b : DEVICE_WEB_PASSWORD_HASH) e = (e ^ b) * 0x01000193u;
    return e;
}();

ra::UserRecord provisioning_admin() noexcept {
    ra::UserRecord u{};
    constexpr std::string_view name = DEVICE_WEB_ADMIN_USER;
    std::copy(name.begin(), name.end(), u.name.begin());
    u.role = Role::Admin;
    u.flags = ra::UserRecord::must_change;
    u.password.iterations = DEVICE_WEB_PBKDF2_ITERATIONS;
    std::memcpy(u.password.salt.data(), DEVICE_WEB_PASSWORD_SALT, ra::salt_bytes);
    std::memcpy(u.password.hash.data(), DEVICE_WEB_PASSWORD_HASH, ra::hash_bytes);
    return u;
}

Users users;  // network thread
// the copy the log thread writes: staged under the kernel lock
Users staged;
std::atomic<bool> save_pending{false};

void request_save() noexcept {
    {
        const Kernel::Lock lock;
        staged = users;
    }
    save_pending = true;
}

// ---- sessions, throttle, presence ---------------------------------------------------------------------
bool random_bytes(std::span<std::byte> out) noexcept {
    for (std::size_t i = 0; i < out.size(); i += 4) {
        const std::uint32_t r = board::Rng::next();
        for (std::size_t j = 0; j < 4 && i + j < out.size(); ++j) out[i + j] = static_cast<std::byte>(r >> (8 * j));
    }
    return true;
}

ra::SessionTable<max_logins> sessions{&random_bytes};
ra::LoginThrottle<16> throttle;
ra::PresenceButton button;
std::atomic<std::uint32_t> button_pressed_ms{0};
std::atomic<bool> button_pressed{false};

std::uint32_t now_ms() noexcept { return kernel.now(); }

bool present(bool local) noexcept {
    if (button_pressed.exchange(false)) button.pressed(button_pressed_ms);
    return button.present(local, now_ms());
}

/// A WebSocket's login: bound at the upgrade.
struct Binding {
    ra::SessionHandle login{};
    bool local = false;
};
std::array<Binding, net::max_https_sessions> bindings{};

// ---- audit ----------------------------------------------------------------------------------------------
struct Ip {
    char text[16];
};
Ip ip_text(rtos::net::Ipv4Address a) noexcept {
    Ip t{};
    std::snprintf(t.text, sizeof t.text, "%u.%u.%u.%u", static_cast<unsigned>(a.value >> 24), static_cast<unsigned>((a.value >> 16) & 0xFF),
                  static_cast<unsigned>((a.value >> 8) & 0xFF), static_cast<unsigned>(a.value & 0xFF));
    return t;
}
const char* via(bool local) noexcept { return local ? "usb" : "lan"; }

// ---- JSON (the API's tiny bodies) -----------------------------------------------------------------------

/// The string value of "key" in a flat JSON object, unescaped into `out` (\" \\ \/ only).
std::optional<std::string_view> json_string(std::string_view body, std::string_view key, std::span<char> out) noexcept {
    std::array<char, 24> quoted{};
    if (key.size() + 2 > quoted.size()) return std::nullopt;
    quoted[0] = '"';
    std::copy(key.begin(), key.end(), quoted.begin() + 1);
    quoted[key.size() + 1] = '"';
    const std::string_view q(quoted.data(), key.size() + 2);
    std::size_t at = body.find(q);
    if (at == std::string_view::npos) return std::nullopt;
    at += q.size();
    while (at < body.size() && (body[at] == ' ' || body[at] == ':' || body[at] == '\t')) ++at;
    if (at >= body.size() || body[at] != '"') return std::nullopt;
    std::size_t n = 0;
    for (++at; at < body.size(); ++at) {
        char c = body[at];
        if (c == '"') return std::string_view(out.data(), n);
        if (c == '\\') {
            if (++at >= body.size()) return std::nullopt;
            c = body[at];
            if (c != '"' && c != '\\' && c != '/') return std::nullopt;  // \uXXXX etc.: not for passwords here
        }
        if (static_cast<unsigned char>(c) < 0x20 || n == out.size()) return std::nullopt;
        out[n++] = c;
    }
    return std::nullopt;
}

void json_error(ApiResponse& out, std::string_view status, std::string_view message, std::uint32_t retry_s = 0) noexcept {
    out.status = status;
    out.retry_after_s = retry_s;
    out.write(R"({"error":")");
    out.write(message);
    out.write("\"");
    if (retry_s != 0) {
        std::array<char, 16> n{};
        const int k = std::snprintf(n.data(), n.size(), "%lu", static_cast<unsigned long>(retry_s));
        out.write(R"(,"retryAfter":)");
        out.write({n.data(), static_cast<std::size_t>(k)});
    }
    out.write("}");
}

void json_session(ApiResponse& out, const ra::SessionInfo& s, bool local) noexcept {
    std::array<char, 200> j{};
    const int k = std::snprintf(j.data(), j.size(),
                                R"({"user":"%.*s","role":"%.*s","mustChangePassword":%s,"present":%s,"via":"%s","idleTimeoutS":%lu})",
                                static_cast<int>(users.users[s.user].user_name().size()), users.users[s.user].user_name().data(),
                                static_cast<int>(ra::role_name(s.role).size()), ra::role_name(s.role).data(),
                                s.must_change_password ? "true" : "false", present(local) ? "true" : "false", via(local),
                                static_cast<unsigned long>(sessions.policy().idle_ms / 1000));
    out.write({j.data(), static_cast<std::size_t>(std::max(k, 0))});
}

// ---- the session of a request -----------------------------------------------------------------------------
bool is_local(const ApiRequest& r) noexcept { return r.interface == net::usb_interface; }

std::optional<ra::SessionHandle> login_of(const ApiRequest& r) noexcept {
    const auto cookie = r.header("Cookie");
    if (!cookie) return std::nullopt;
    const auto token = ra::cookie_value(*cookie, cookie_name);
    if (!token) return std::nullopt;
    return sessions.find(*token, now_ms());
}

ra::Subject subject(ra::SessionHandle h, bool local) noexcept {
    const ra::SessionInfo* s = sessions.info(h);
    if (s == nullptr) return {};
    return {.role = s->role, .must_change_password = s->must_change_password, .present = present(local || s->local)};
}

// ---- background work: a password check or hash per HTTPS session -----------------------------------------
enum class Op : std::uint8_t { None, Login, ChangeVerify, ChangeHash, SetHash };
struct Pending {
    Op op = Op::None;
    std::uint8_t user = 0;
    std::uint8_t target = 0;  ///< SetHash: the account that gets the new password
    bool local = false;
    rtos::net::Ipv4Address peer{};
    std::optional<ra::SessionHandle> login;  ///< the requester's session (password change)
    ra::PasswordCheck check;
    ra::PasswordSetter setter;
    std::array<char, ra::max_password_bytes> next{};  ///< ChangeVerify: the new password, until hashed
    std::size_t next_size = 0;
    std::array<char, ra::max_user_name + 1> requested{};  ///< Login: the name as sent (audit)
    bool must_change_after = false;
    void clear() noexcept {
        check.clear();
        setter.clear();
        volatile char* p = next.data();
        for (std::size_t i = 0; i < next.size(); ++i) p[i] = 0;
        next_size = 0;
        requested = {};
        op = Op::None;
        login.reset();
    }
};
std::array<Pending, net::max_https_sessions> pending{};

void finish_login(Pending& p, ApiResponse& out) noexcept {
    const bool ok = p.check.matches();
    const ra::UserRecord& u = users.users[p.user];
    const Ip ip = ip_text(p.peer);
    if (!ok) {
        throttle.failure(p.peer.value, p.local, now_ms());
        RTOS_LOG_WARN("auth: login of %s from %s (%s) failed (%lu failures from there)", p.requested.data(), ip.text, via(p.local),
                      static_cast<unsigned long>(throttle.failures(p.peer.value)));
        const std::uint32_t wait = throttle.wait_ms(p.peer.value, p.local, now_ms());
        json_error(out, "401 Unauthorized", "invalid user or password", (wait + 999) / 1000);
        return;
    }
    throttle.success(p.peer.value);
    ra::Token token{};
    const ra::SessionInfo info{.user = p.user, .role = u.role, .must_change_password = u.must_change_password(), .local = p.local};
    const auto h = sessions.create(info, now_ms(), token);
    if (!h) {
        json_error(out, "503 Service Unavailable", "no random numbers");
        return;
    }
    std::array<char, ApiResponse::max_cookie> c{};
    const int k = std::snprintf(c.data(), c.size(), "%.*s=%.*s; HttpOnly; Secure; SameSite=Strict; Path=/", static_cast<int>(cookie_name.size()),
                                cookie_name.data(), static_cast<int>(token.size()), token.data());
    out.set_cookie({c.data(), static_cast<std::size_t>(k)});
    json_session(out, *sessions.info(*h), p.local);
    RTOS_LOG_INFO("auth: %.*s logged in from %s (%s) as %.*s%s", static_cast<int>(u.user_name().size()), u.user_name().data(), ip.text,
                  via(p.local), static_cast<int>(ra::role_name(u.role).size()), ra::role_name(u.role).data(),
                  u.must_change_password() ? ", must change the password" : "");
}

/// The new password is hashed: store it, end the account's other logins.
void finish_set(Pending& p, ApiResponse& out) noexcept {
    const auto record = p.setter.record();
    if (!record) {
        json_error(out, "500 Internal Server Error", "hashing failed");
        return;
    }
    ra::UserRecord& u = users.users[p.target];
    u.password = *record;
    u.flags = p.must_change_after ? ra::UserRecord::must_change : 0;
    request_save();
    const std::size_t ended = sessions.remove_user(p.target, p.login.value_or(ra::SessionHandle{}));
    if (p.login) {
        if (ra::SessionInfo* s = sessions.info(*p.login); s != nullptr && s->user == p.target) {
            s->must_change_password = false;  // the session that changed its own password goes on
        }
    }
    RTOS_LOG_INFO("auth: password of %.*s %s by %.*s from %s (%s), %u other session(s) ended", static_cast<int>(u.user_name().size()),
                  u.user_name().data(), p.op == Op::ChangeHash ? "changed" : "set",
                  static_cast<int>(users.users[p.user].user_name().size()), users.users[p.user].user_name().data(),
                  ip_text(p.peer).text, via(p.local), static_cast<unsigned>(ended));
    out.write(R"({"ok":true})");
}

ApiResult step(void*, SessionId id, ApiResponse& out) noexcept {
    Pending& p = pending[id];
    switch (p.op) {
        case Op::Login:
        case Op::ChangeVerify:
            if (p.check.step(iterations_per_step)) return ApiResult::Pending;
            if (p.op == Op::Login) {
                finish_login(p, out);
                break;
            }
            if (!p.check.matches()) {
                throttle.failure(p.peer.value, p.local, now_ms());
                RTOS_LOG_WARN("auth: password change of %.*s from %s: wrong current password",
                              static_cast<int>(users.users[p.user].user_name().size()), users.users[p.user].user_name().data(),
                              ip_text(p.peer).text);
                json_error(out, "403 Forbidden", "current password wrong");
                break;
            }
            if (!p.setter.start({p.next.data(), p.next_size}, DEVICE_WEB_PBKDF2_ITERATIONS, &random_bytes)) {
                json_error(out, "500 Internal Server Error", "hashing failed");
                break;
            }
            p.op = Op::ChangeHash;
            return ApiResult::Pending;
        case Op::ChangeHash:
        case Op::SetHash:
            if (p.setter.step(iterations_per_step)) return ApiResult::Pending;
            finish_set(p, out);
            break;
        case Op::None: json_error(out, "500 Internal Server Error", "nothing pending"); break;
    }
    p.clear();
    return ApiResult::Done;
}

void cancel(void*, SessionId id) noexcept { pending[id].clear(); }

// ---- endpoints ----------------------------------------------------------------------------------------------
using Password = std::array<char, ra::max_password_bytes + 1>;
using Name = std::array<char, ra::max_user_name + 1>;

ApiResult login(const ApiRequest& r, ApiResponse& out) noexcept {
    const bool local = is_local(r);
    if (const std::uint32_t wait = throttle.wait_ms(r.peer.value, local, now_ms()); wait != 0) {
        RTOS_LOG_WARN("auth: login from %s (%s) throttled for %lu ms", ip_text(r.peer).text, via(local), static_cast<unsigned long>(wait));
        json_error(out, "429 Too Many Requests", "too many failed logins", (wait + 999) / 1000);
        return ApiResult::Done;
    }
    Name name_buf{};
    Password pw_buf{};
    const auto name = json_string(r.body, "user", name_buf);
    const auto pw = json_string(r.body, "password", pw_buf);
    if (!name || !pw || pw->empty()) {
        json_error(out, "400 Bad Request", "user and password expected");
        return ApiResult::Done;
    }
    // session fixation: a login presented with the request ends here, the new one gets a new token
    if (const auto old = login_of(r)) sessions.remove(*old);
    Pending& p = pending[r.id];
    p.clear();
    p.peer = r.peer;
    p.local = local;
    // An unknown user (or one whose password is not set yet) costs the same PBKDF2 as a known one -
    // no user enumeration by timing: a password that cannot match is checked against the record of
    // the first account (control characters never pass password_problem()).
    int user = -1;
    for (std::size_t i = 0; i < max_users; ++i) {
        if (users.users[i].used() && users.users[i].user_name() == *name && users.users[i].password.iterations != 0) {
            user = static_cast<int>(i);
        }
    }
    const auto first = std::ranges::find_if(users.users, [](const ra::UserRecord& u) { return u.used() && u.password.iterations != 0; });
    if (first == users.users.end()) {
        json_error(out, "503 Service Unavailable", "no account");
        return ApiResult::Done;
    }
    const bool known = user >= 0;
    p.user = static_cast<std::uint8_t>(known ? user : first - users.users.begin());
    std::copy_n(name->begin(), std::min(name->size(), ra::max_user_name), p.requested.begin());
    const bool started = known ? p.check.start(*pw, users.users[p.user].password)
                               : p.check.start("\x01\x02 no such user \x02\x01", first->password);
    std::fill(pw_buf.begin(), pw_buf.end(), '\0');
    if (!started) {
        throttle.failure(r.peer.value, local, now_ms());
        json_error(out, "401 Unauthorized", "invalid user or password");
        p.clear();
        return ApiResult::Done;
    }
    p.op = Op::Login;
    return ApiResult::Pending;
}

ApiResult logout(const ApiRequest& r, ApiResponse& out) noexcept {
    if (const auto h = login_of(r)) {
        const ra::SessionInfo* s = sessions.info(*h);
        RTOS_LOG_INFO("auth: %.*s logged out from %s", static_cast<int>(users.users[s->user].user_name().size()),
                      users.users[s->user].user_name().data(), ip_text(r.peer).text);
        sessions.remove(*h);  // its WebSocket closes at the next check (service())
    }
    std::array<char, ApiResponse::max_cookie> c{};
    const int k = std::snprintf(c.data(), c.size(), "%.*s=; HttpOnly; Secure; SameSite=Strict; Path=/; Max-Age=0",
                                static_cast<int>(cookie_name.size()), cookie_name.data());
    out.set_cookie({c.data(), static_cast<std::size_t>(k)});
    out.write(R"({"ok":true})");
    return ApiResult::Done;
}

ApiResult session_info(const ApiRequest& r, ApiResponse& out) noexcept {
    const auto h = login_of(r);
    if (!h) {
        json_error(out, "401 Unauthorized", "not logged in");
        return ApiResult::Done;
    }
    json_session(out, *sessions.info(*h), is_local(r) || sessions.info(*h)->local);
    return ApiResult::Done;
}

/// POST /api/password {"current","new"}: own password (the only thing a must-change login may do).
ApiResult change_password(const ApiRequest& r, ApiResponse& out) noexcept {
    const auto h = login_of(r);
    if (!h) {
        json_error(out, "401 Unauthorized", "not logged in");
        return ApiResult::Done;
    }
    const ra::SessionInfo& s = *sessions.info(*h);
    Password cur_buf{}, new_buf{};
    const auto current = json_string(r.body, "current", cur_buf);
    const auto next = json_string(r.body, "new", new_buf);
    if (!current || !next) {
        json_error(out, "400 Bad Request", "current and new password expected");
        return ApiResult::Done;
    }
    if (const auto problem = ra::password_problem(*next, users.users[s.user].user_name())) {
        json_error(out, "400 Bad Request", *problem);
        return ApiResult::Done;
    }
    if (*next == *current) {
        json_error(out, "400 Bad Request", "the new password equals the current one");
        return ApiResult::Done;
    }
    Pending& p = pending[r.id];
    p.clear();
    p.user = p.target = s.user;
    p.peer = r.peer;
    p.local = is_local(r);
    p.login = h;
    p.must_change_after = false;
    std::copy(next->begin(), next->end(), p.next.begin());
    p.next_size = next->size();
    if (!p.check.start(*current, users.users[s.user].password)) {
        json_error(out, "400 Bad Request", "current password wrong");
        p.clear();
        return ApiResult::Done;
    }
    std::fill(cur_buf.begin(), cur_buf.end(), '\0');
    std::fill(new_buf.begin(), new_buf.end(), '\0');
    p.op = Op::ChangeVerify;
    return ApiResult::Pending;
}

/// Admin: GET /api/users (list), POST /api/users {"action":"add|delete|role|password", ...}.
ApiResult manage_users(const ApiRequest& r, ApiResponse& out) noexcept {
    const auto h = login_of(r);
    const bool local = is_local(r);
    const ra::Subject who = h ? subject(*h, local) : ra::Subject{};
    const std::string_view action_text = [&] {
        static Name a{};
        return json_string(r.body, "action", a).value_or("");
    }();
    // setting someone else's password is a password reset: physical presence
    const ra::Permission need = action_text == "password" ? ra::permission::admin_critical : ra::permission::admin;
    if (const Decision d = ra::decide(who, need); d != Decision::Allowed) {
        RTOS_LOG_WARN("auth: user management (%s) from %s refused: %.*s", r.method == "GET" ? "list" : "change",
                      ip_text(r.peer).text, static_cast<int>(ra::decision_name(d).size()), ra::decision_name(d).data());
        json_error(out, d == Decision::NotAuthenticated ? "401 Unauthorized" : "403 Forbidden", ra::decision_name(d));
        return ApiResult::Done;
    }
    const ra::SessionInfo& me = *sessions.info(*h);
    if (r.method == "GET") {
        out.write("{\"users\":[");
        bool first = true;
        for (const auto& u : users.users) {
            if (!u.used()) continue;
            std::array<char, 96> j{};
            const int k = std::snprintf(j.data(), j.size(), R"(%s{"user":"%.*s","role":"%.*s","mustChangePassword":%s})", first ? "" : ",",
                                        static_cast<int>(u.user_name().size()), u.user_name().data(),
                                        static_cast<int>(ra::role_name(u.role).size()), ra::role_name(u.role).data(),
                                        u.must_change_password() ? "true" : "false");
            out.write({j.data(), static_cast<std::size_t>(std::max(k, 0))});
            first = false;
        }
        out.write("]}");
        return ApiResult::Done;
    }
    Name name_buf{}, role_buf{};
    Password pw_buf{};
    const auto name = json_string(r.body, "user", name_buf);
    if (!name || !ra::valid_user_name(*name)) {
        json_error(out, "400 Bad Request", "user name: 1-15 of a-z 0-9 . _ -");
        return ApiResult::Done;
    }
    const auto idx = users.find(*name);
    const auto role = json_string(r.body, "role", role_buf).and_then([](std::string_view s) { return ra::parse_role(s); });
    const auto me_name = users.users[me.user].user_name();
    if (action_text == "add" || action_text == "password") {
        const auto pw = json_string(r.body, "password", pw_buf);
        if (!pw) {
            json_error(out, "400 Bad Request", "password expected");
            return ApiResult::Done;
        }
        if (const auto problem = ra::password_problem(*pw, *name)) {
            json_error(out, "400 Bad Request", *problem);
            return ApiResult::Done;
        }
        std::optional<std::uint8_t> target = idx;
        if (action_text == "add") {
            if (idx) return json_error(out, "409 Conflict", "user exists"), ApiResult::Done;
            if (!role) return json_error(out, "400 Bad Request", "role: observer, operator, programmer or admin"), ApiResult::Done;
            target = users.free_slot();
            if (!target) return json_error(out, "507 Insufficient Storage", "no free account"), ApiResult::Done;
            ra::UserRecord& u = users.users[*target];
            u = {};
            std::copy(name->begin(), name->end(), u.name.begin());
            u.role = *role;
            u.password.iterations = 0;  // no login until the hash is there
            RTOS_LOG_INFO("auth: %.*s added user %.*s (%.*s)", static_cast<int>(me_name.size()), me_name.data(), static_cast<int>(name->size()),
                          name->data(), static_cast<int>(ra::role_name(*role).size()), ra::role_name(*role).data());
        } else if (!idx) {
            return json_error(out, "404 Not Found", "no such user"), ApiResult::Done;
        }
        Pending& p = pending[r.id];
        p.clear();
        p.user = me.user;
        p.target = *target;
        p.peer = r.peer;
        p.local = local;
        p.login = h;
        p.must_change_after = *target != me.user;  // a password set by someone else is a one-time password
        if (!p.setter.start(*pw, DEVICE_WEB_PBKDF2_ITERATIONS, &random_bytes)) {
            p.clear();
            return json_error(out, "500 Internal Server Error", "hashing failed"), ApiResult::Done;
        }
        std::fill(pw_buf.begin(), pw_buf.end(), '\0');
        p.op = Op::SetHash;
        return ApiResult::Pending;
    }
    if (!idx) return json_error(out, "404 Not Found", "no such user"), ApiResult::Done;
    ra::UserRecord& u = users.users[*idx];
    if (action_text == "delete") {
        if (*idx == me.user) return json_error(out, "409 Conflict", "an admin cannot delete the own account"), ApiResult::Done;
        if (u.role == Role::Admin && users.admins() == 1) return json_error(out, "409 Conflict", "the last admin"), ApiResult::Done;
        RTOS_LOG_INFO("auth: %.*s deleted user %.*s", static_cast<int>(me_name.size()), me_name.data(), static_cast<int>(name->size()), name->data());
        sessions.remove_user(*idx);
        u = {};
        request_save();
        out.write(R"({"ok":true})");
        return ApiResult::Done;
    }
    if (action_text == "role") {
        if (!role) return json_error(out, "400 Bad Request", "role: observer, operator, programmer or admin"), ApiResult::Done;
        if (u.role == Role::Admin && *role != Role::Admin && users.admins() == 1) {
            return json_error(out, "409 Conflict", "the last admin"), ApiResult::Done;
        }
        RTOS_LOG_INFO("auth: %.*s changed the role of %.*s: %.*s -> %.*s", static_cast<int>(me_name.size()), me_name.data(),
                      static_cast<int>(name->size()), name->data(), static_cast<int>(ra::role_name(u.role).size()), ra::role_name(u.role).data(),
                      static_cast<int>(ra::role_name(*role).size()), ra::role_name(*role).data());
        u.role = *role;
        sessions.remove_user(*idx);  // logins keep the role they had: log in again
        request_save();
        out.write(R"({"ok":true})");
        return ApiResult::Done;
    }
    json_error(out, "400 Bad Request", "action: add, delete, role or password");
    return ApiResult::Done;
}

/// POST /api/recover: back to the label password of the provisioning account. Physical presence
/// only (USB or the button); no login needed - this is the way back for a forgotten password.
ApiResult recover(const ApiRequest& r, ApiResponse& out) noexcept {
    const bool local = is_local(r);
    if (!present(local)) {
        RTOS_LOG_WARN("auth: password recovery from %s (%s) refused: no physical presence", ip_text(r.peer).text, via(local));
        json_error(out, "403 Forbidden", decision_name(Decision::PresenceRequired));
        return ApiResult::Done;
    }
    const ra::UserRecord admin = provisioning_admin();
    const auto idx = users.find(admin.user_name());
    const std::uint8_t at = idx.value_or(users.free_slot().value_or(0));
    users.users[at] = admin;
    for (std::uint8_t i = 0; i < max_users; ++i) sessions.remove_user(i);
    button.consume();
    request_save();
    RTOS_LOG_WARN("auth: password of %.*s reset to the label password from %s (%s); every session ended",
                  static_cast<int>(admin.user_name().size()), admin.user_name().data(), ip_text(r.peer).text, via(local));
    out.write(R"({"ok":true})");
    return ApiResult::Done;
}

ApiResult begin(void*, const ApiRequest& r, ApiResponse& out) noexcept {
    const bool post = r.method == "POST";
    if (r.path == "/api/login" && post) return login(r, out);
    if (r.path == "/api/logout" && post) return logout(r, out);
    if (r.path == "/api/session") return session_info(r, out);
    if (r.path == "/api/password" && post) return change_password(r, out);
    if (r.path == "/api/users") return manage_users(r, out);
    if (r.path == "/api/recover" && post) return recover(r, out);
    json_error(out, "404 Not Found", "no such API");
    return ApiResult::Done;
}

/// The WebSocket upgrade: a valid login that need not change its password first.
bool authorize_upgrade(void*, const ApiRequest& r) noexcept {
    const auto h = login_of(r);
    const ra::SessionInfo* s = h ? sessions.info(*h) : nullptr;
    if (s == nullptr || s->must_change_password) {
        RTOS_LOG_WARN("auth: WebSocket from %s (%s) refused: %s", ip_text(r.peer).text, via(is_local(r)),
                      s == nullptr ? "not logged in" : "password change required");
        return false;
    }
    bindings[r.id] = {.login = *h, .local = is_local(r)};
    return true;
}

const Server::ApiHandler handler{.context = nullptr, .prefix = "/api/", .begin = &begin, .step = &step, .cancel = &cancel,
                                 .authorize_upgrade = &authorize_upgrade};

Service service_instance;
rtos::Tick next_check = 0;

}  // namespace

void init() noexcept {
    std::uint32_t epoch = 0;
    Users loaded{};
    if (store.load(loaded, epoch) && epoch == provisioning_epoch && loaded.admins() != 0) {
        users = loaded;
        RTOS_LOG_INFO("auth: %u account(s) from the flash", static_cast<unsigned>(std::ranges::count_if(users.users, [](const auto& u) { return u.used(); })));
    } else {
        users = {};
        users.users[0] = provisioning_admin();
        request_save();
        RTOS_LOG_INFO("auth: provisioning account %s with the label password (%s)", DEVICE_WEB_ADMIN_USER,
                      epoch != provisioning_epoch && epoch != 0 ? "new provisioning" : "first start");
    }
}

const Server::ApiHandler& api_handler() noexcept { return handler; }

Decision authorize(SessionId id, Permission need, const char* what) noexcept {
    Binding& b = bindings[id];
    ra::Subject who{};
    if (sessions.touch(b.login, now_ms())) who = subject(b.login, b.local);
    const Decision d = ra::decide(who, need);
    const ra::SessionInfo* s = sessions.info(b.login);
    const std::string_view name = s != nullptr ? users.users[s->user].user_name() : std::string_view("-");
    if (d != Decision::Allowed) {
        RTOS_LOG_WARN("auth: %s by %.*s refused: %.*s", what, static_cast<int>(name.size()), name.data(),
                      static_cast<int>(ra::decision_name(d).size()), ra::decision_name(d).data());
    } else if (need.presence || need.least >= Role::Programmer) {
        RTOS_LOG_INFO("auth: %s by %.*s (%s)", what, static_cast<int>(name.size()), name.data(), via(b.local || (s != nullptr && s->local)));
    }
    if (d == Decision::Allowed && need.presence) button.consume();  // one press, one critical action
    return d;
}

void websocket_closed(SessionId id) noexcept { bindings[id] = {}; }

Role role_of(SessionId id) noexcept {
    const ra::SessionInfo* s = sessions.info(bindings[id].login);
    return s != nullptr ? s->role : Role::None;
}

void Service::service() noexcept {
    const rtos::Tick now = kernel.now();
    if (!rtos::reached(now, next_check)) return;
    next_check = now + check_period_ms;
    sessions.expire(now);
    for (std::size_t i = 0; i < bindings.size(); ++i) {
        if (bindings[i].login.valid() && !sessions.alive(bindings[i].login, now)) {
            RTOS_LOG_INFO("auth: WebSocket %u closed: its login ended", static_cast<unsigned>(i));
            bindings[i] = {};
            (void)net::server.close(static_cast<SessionId>(i), 1008);  // policy violation: log in again
        }
    }
}

std::uint32_t Service::next_deadline() const noexcept { return next_check; }

Service& service() noexcept { return service_instance; }

void store_pending_changes() noexcept {
    if (!save_pending.exchange(false)) return;
    static Users copy;
    {
        const Kernel::Lock lock;
        copy = staged;
    }
    const std::uint32_t t0 = kernel.now();
    flash_lock.take();
    const bool ok = store.save(copy, provisioning_epoch);
    flash_lock.give();
    if (ok) {
        RTOS_LOG_INFO("auth: accounts saved (%lu ms)", static_cast<unsigned long>(kernel.now() - t0));
    } else {
        RTOS_LOG_ERROR("auth: accounts could not be written to the flash");
    }
}

void presence_pressed() noexcept {
    button_pressed_ms = kernel.now();
    button_pressed = true;
}

}  // namespace fcu::auth
