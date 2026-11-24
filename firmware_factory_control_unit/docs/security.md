# Security of the web UI: login, roles, presence, audit

The building blocks are rtoos' (`rtoos/include/rtos/auth/`, `rtoos/docs/security.md`); this document is the
policy of the Factory Control Unit (`src/auth.hpp`, `src/auth.cpp`, `src/ws_permissions.hpp`).

## Provisioning: one password per device

- The builder generates a one-time password per board when it first provisions it
  (`GenerateWebCredentials`, part of every pipeline): 16 characters in groups of four from an alphabet
  without look-alikes (about 92 bits). It is stored in the **board archive** next to the board's other data:
  `<BoardStorage:BoardsDir>/<board id>/board_info.json`, field `webAdminPassword` (keyed by the chip UID like
  everything there).
- Every later build reuses it - the label stays valid. A new one only with
  `WebAuth:ForceRecreateOneTimeWebUiPassword: true` in `builder/appsettings.json` or
  `--force-recreate-one-time-web-ui-password` on the command line; then print a new label.
- The firmware gets only the PBKDF2-HMAC-SHA256 record (`generated/device_credentials.hh`, gitignored):
  user `admin`, iterations (`WebAuth:Pbkdf2Iterations`, 6000), salt (derived from chip UID and password,
  so the record changes exactly when the password does), hash. No two devices share a password and there is
  no default password (ETSI EN 303 645 provision 5.1, CRA Annex I).
- Every creation is recorded in `<BoardsDir>/provisioning.csv` (board ID, chip UID, hostname, MAC,
  revision, date, event - never the password).
- The label (`build/labels/<hostname>.zpl/.png/.pdf`, gitignored) shows hostname, board ID (chip UID), MAC,
  hardware revision, provisioning date, the password and a QR code with `https://<hostname>.local/` (the URL
  only). ZPL for a Zebra-type printer, 51 x 25 mm at 300 dpi by default (`LabelPrinter:WidthMm/HeightMm/Dpi`);
  with `LabelPrinter:Host` (and `Port`, 9100) the builder sends it raw to the printer, otherwise it only
  writes the files. `PrintLabel` prints it again. Brother QL-700 (62 mm tape) and an A4 sheet with cut
  marks for any printer (`PrintLabelSheet`): [labels.md](labels.md).

## On the device

- Accounts: up to 8, in two flash sectors right below the PLC's program slots (A/B, `CredentialStore`;
  2 x 8 KB on the STM32H563, 2 x 128 KB on the STM32H743; `FLASH_DATA_START` 0x08180000). The log thread
  writes them (a flash erase may block; the network thread must not), under `flash_lock`, which the PLC
  thread takes for its slots too. A new provisioning (new label password) replaces the stored table.
- First login with the label password: the session may do nothing but `POST /api/password`; the WebSocket
  is refused until the password is changed. New passwords: 10-64 bytes, not the user name.
- API (`/api/`, JSON): `POST login {user,password}`, `POST logout`, `GET session`,
  `POST password {current,new}`, `GET users` / `POST users {action: add|delete|role|password, ...}` (admin),
  `POST recover` (physical presence, no login: back to the label password).
- Session: cookie `fcu_session` (`HttpOnly; Secure; SameSite=Strict; Path=/`), 256-bit random token, 30 min
  inactivity, 12 h lifetime, at most 6 logins (least recently used goes). A login never keeps a token it was
  sent (session fixation). Logout, a password or role change ends the logins concerned; their WebSockets are
  closed with 1008 within a second.
- The static web app loads without login. The WebSocket upgrade needs a valid login **and** the Origin check;
  cross-origin `POST`s are refused (403).
- Password checks run as background steps of the network thread (32 PBKDF2 iterations, about 0.9 ms each):
  Modbus, OPC UA and the other browsers keep being served during a login.
- Throttling per client address: 3 free failures, then 1, 2, 4 ... 60 s, lockout of 5 min after 10; globally
  30 failures per minute lock remote logins for a minute (USB logins are exempt). Refused attempts get 429 with
  `Retry-After` at once, without a password check. An unknown user costs the same check as a known one.

## Roles

Checked on the device for every WebSocket request (`src/ws_permissions.hpp`, host test
`tests/host/ws_permissions_test.cpp`); the web UI only hides the workspaces a role cannot use. A request the
table does not know needs the admin role.

| Role | May |
|---|---|
| Beobachter (observer) | read: values, status, PLC status/watch/project, missions, task list, log |
| Bediener (operator) | + write registers, actuators, steppers, RoArm (jog, teach, play) |
| Programmierer (programmer) | + PLC project, RUN/STOP, online changes, debugging, unforce, missions and axis settings; PLC download and forcing with presence |
| Administrator (admin) | + accounts, terminal; password reset of others with presence |

Refusals answer with the WebSocket error codes 0x101 (not logged in), 0x102 (role), 0x103 (presence),
0x104 (password change first). The role model is protocol-independent (`rtos::auth::decide`), ready for
OPC UA user tokens and SSH logins.

## Physical presence

PLC download and commit, forcing, password reset of another user and recovery need proof that the person
is at the machine: the session came in over the **USB CDC-NCM interface** (the TCP connection's arrival
interface, not its address), or a presence button was pressed in the last two minutes. No FCU revision has a
button for it yet (`src/hal/rev*/pins.hpp`), so today it is USB; `auth::presence_pressed()` is the hook for
one. Firmware update and certificate changes have no web API yet; they will get `admin_critical`.

## Audit

Every login, failure, throttling, logout, password, role and account change, refused WebSocket request and
every programmer/critical request goes into the log journal as an `auth:` line with user, address and
interface (console, the Terminal page).

## Tests

- rtoos host tests: `auth_test.cpp`, `Http.*Body*`, `HttpsApi.*` (see rtoos docs/security.md).
- `tests/host/ws_permissions_test.cpp`: the role table.
- `web/src/core/auth.test.ts`: the UI's session mirror.
- `tools/test-auth-renode.py`: end to end in Renode (label password, forced change, cookie flags, roles,
  presence, forged/old tokens, fixation, cross-origin, throttling, audit lines).
- `tools/pentest-renode.py`, group "login": brute force, token forgery, escalation by an observer, malformed
  login bodies; the WebSocket attacks now run logged in.
