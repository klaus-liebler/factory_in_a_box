
## <b>Nx_WebServer Application Description</b>

Firmware for the Control-Unit board (STM32H573, Azure RTOS ThreadX/NetX Duo) that exposes
its Modbus register map (digital I/O, PWM outputs, sensors, USB-PD status, diagnostics --
see `register-map.json`) both as a Modbus TCP server (port 502) and as a small built-in web
UI served directly from flash by the NetX Duo HTTP server over **HTTPS only** (port 443, no
plaintext HTTP) with a per-board certificate (see "Certificate provisioning" below). The
board also publishes its hostname (`factory-box-<6 hex digits>`, matching the certificate's
CN) via mDNS as `<hostname>.local`.

The main entry function `tx_application_define()` is called by ThreadX during kernel start;
all NetX/FileX resources and application threads are created there. `app_main_thread_entry()`
is the only thread started automatically (`TX_AUTO_START`); it performs the initialization
steps that need a running thread context (FileX media open, HTTP server start, DHCP start,
Modbus server init) and then resumes the remaining threads in a fixed order:

 + **led_thread** -- toggles the green status LED (driven by the `/LedOn` / `/LedOff` HTTP
   demo endpoints)
 + **link_thread** -- watches Ethernet link up/down and re-triggers DHCP on reconnect
 + **modbus_server_thread** -- runs the Modbus TCP server (`ModbusTcpServer::run()`)
 + **io_thread** -- polls digital inputs (lightbarriers), the analog pressure sensor, and
   drives the pneumatic valves / motor PWM outputs from the Modbus holding registers
 + **usb_pd_thread** -- runs the USB-PD sink state machine (`PowerSink.Loop()`) and pushes
   the negotiated voltage/current into the Modbus register map

The NetX Duo HTTP server's request callback (`webserver_request_callback` in `app.cpp`)
intercepts a handful of routes before falling back to FileX/SD-card file serving:

 + `GET /` -- serves the gzip-compressed, single-file register web UI that is compiled
   directly into flash (see "Web UI" below), streamed in small chunks so the (deliberately
   tiny) HTTP server packet pool never needs to hold more than one packet at a time. Only
   reachable over HTTPS (port 443) -- there is no plaintext HTTP fallback.
 + `GET /api/registers` -- plain-text dump of all Modbus register values (comma-separated,
   fixed-width; no JSON parser needed on either side).
 + `GET /api/write-holding?address=N&value=V` -- writes a single holding register.
 + `/GetNetInfo`, `/LedOn`, `/LedOff` -- small demo endpoints kept from the original NetX
   Duo example this project is based on.

#### <b>Expected success behavior</b>

 + The board's DHCP-assigned IP address is printed on the serial console.
 + Opening `https://<board-ip>/` (or `https://<hostname>.local/` once mDNS has resolved)
   in a browser shows the "Modbus Register Console" web UI, auto-refreshing every second;
   toggling a valve switch or dragging a PWM slider writes the corresponding holding
   register immediately. The browser will show a certificate-trust warning until the
   signing CA (`rootCA.pem.crt`) has been imported client-side -- see "Certificate
   provisioning" below.
 + `tools/test_modbus.py --host <board-ip>` can read/write the same registers over Modbus
   TCP independently of the web UI.

#### <b>Error behaviors</b>

If the HTTP server fails to start, or FileX media open fails (missing/unformatted SD card),
the application calls `Error_Handler()`, which blinks the red LED and halts. If the Modbus
TCP server fails to initialize, the same applies.

#### <b>Assumptions if any</b>

 + A formatted SD card must be inserted before starting the application -- FileX still
   requires `fx_media_open()` to succeed at boot even though the register web UI itself no
   longer needs any files on the card (it is served from flash).
 + The board must be reachable via DHCP on the connected Ethernet network.

#### <b>ThreadX usage hints</b>

 - ThreadX uses the Systick as time base, thus it is mandatory that the HAL uses a separate time base through the TIM IPs.
 - ThreadX is configured with 100 ticks/sec by default, this should be taken into account when using delays or timeouts at application. It is always possible to reconfigure it, by updating the "TX_TIMER_TICKS_PER_SECOND" define in the "tx_user.h" file. The update should be reflected in "tx_initialize_low_level.S" file too.
 - ThreadX is disabling all interrupts during kernel start-up to avoid any unexpected behavior, therefore all system related calls (HAL, BSP) should be done either at the beginning of the application or inside the thread entry functions.
 - ThreadX offers the "tx_application_define()" function, that is automatically called by the tx_kernel_enter() API.
   It is highly recommended to use it to create all applications ThreadX related resources (threads, semaphores, memory pools...) but it should not in any way contain a system API call (HAL or BSP).

#### <b>NetX Duo usage hints</b>

- Depending on the application scenario, the total TX and RX descriptors may need to be increased by updating respectively the "ETH_TX_DESC_CNT" and "ETH_RX_DESC_CNT" in the "stm32h5xx_hal_conf.h", to guarantee the application correct behaviour, but this will cost extra memory to allocate.
- The HTTP server's packet pool (`SERVER_POOL_SIZE` in `app.cpp`) is intentionally tiny (4 packets). Routes that need to send more data than that (`/`, `/api/registers`) stream it in bounded chunks via `send_streamed_response()` instead of building one large response in memory -- see the comments there before enlarging response payloads.
- TLS is provided by NetX Secure + the crypto library, vendored wholesale under `Middlewares/ST/netxduo/{nx_secure,crypto_libraries}` (no `cortex_m33` crypto port exists upstream; the `cortex_m4/gnu` port is reused since it's a pure type-definitions header, no assembly). Scope is deliberately narrowed to **TLS 1.2, ECDSA (P-256), no mutual-TLS/client-certificate verification** to keep the vendored/enabled surface small -- see `Core/Inc/nx_secure_user.h` for the (all-default) NetX Secure config and `nx_web_http_server_secure_configure(...)` in `app.cpp` for the server-side TLS setup. The device key is ECDSA rather than RSA-2048 specifically because RSA modular-exponentiation signing during the `ServerKeyExchange` step is expensive enough on the STM32H573 to need a 16&nbsp;KB HTTP-server thread stack (`SERVER_STACK` in `app.cpp`) just to avoid overflowing -- ECDSA signing is far cheaper, both in CPU time and stack depth.

#### <b>Keywords</b>

RTOS, ThreadX, Network, NetXDuo, Web HTTP Server, FileX, Modbus TCP, USB-PD, SDMMC, UART

#### <b>Hardware and Software environment</b>

  - This application runs on STM32H573xx devices.
  - Developed and tested on an STM32H573I-DK Discovery board (revision MB1520-H573I-C01).
  - USART1 is used for log output: 115200 baud, 8 data bits, 1 stop bit, no parity, no flow control.

## <b>Build</b>

Prerequisites:
 - CMake >= 3.20 and Ninja
 - The `arm-none-eabi` GCC toolchain (e.g. the one bundled with STM32CubeIDE /
   `gnu-tools-for-stm32`)
 - Node.js (only needed if you change anything under `web/` or `register-map.json`)

Build the firmware via the checked-in CMake presets (`CMakePresets.json`, `Debug`/`Release`):

```bash
cmake --preset Debug
cmake --build --preset Debug
```

The resulting binary is `build/Debug/Nx_WebServer.elf`. STM32CubeIDE can also open/build this
project directly via its managed-CMake integration.

### Web UI (`web/`)

The register console UI is a small Vite + Lit single-page app, built as one self-contained,
minified HTML file and compiled straight into the firmware image -- it is **not** part of
the CMake build itself, so a plain firmware build does not require Node.js.

```bash
cd web
npm install          # once
npm run embed        # vite build, then gzip + emit Core/Src/generated/modbus_ui_page.{c,h}
```

`npm run embed` overwrites the generated, checked-in `Core/Src/generated/modbus_ui_page.c/.h`
-- rebuild the firmware afterwards to pick up the change. `npm run dev` starts a local Vite
dev server for UI iteration (requests to `/api/...` only succeed against the real board, so
point a proxy at it or expect those calls to fail while developing purely offline).

### Register map (`register-map.json`)

`register-map.json` is the single source of truth for the Modbus register map (addresses,
units, GPIO wiring, UI widget type). Both `Core/Src/modbus_register_map.hpp` (C++) and
`web/src/register-map.ts` (TypeScript) are generated from it:

```bash
node tools/generate-register-map.mjs
```

After editing `register-map.json`, run the generator, then rebuild both the web UI
(`npm run embed` in `web/`) and the firmware -- neither generated file should be edited by
hand.

### Certificate provisioning (`register-map.json`-independent, once per physical board)

The HTTPS server's certificate is generated (or reused) per board, keyed off the STM32's
96-bit unique ID, and signed by a private CA:

```bash
node tools/provision-certificate.mjs
```

This needs, with the target board connected via ST-Link:
 - STM32CubeProgrammer installed (reads the chip's unique ID over SWD; the script calls the
   full path directly, no PATH entry required)
 - `openssl` on PATH
 - Access to `C:\Users\mail\OneDrive - HSOS\certificates\rootCA.pem.{crt,key}` (the signing
   CA) and `C:\Users\mail\OneDrive - HSOS\stm32_boards\` (per-board certificate cache --
   reused on subsequent runs for the same chip instead of regenerating)

It writes `Core/Src/generated/device_certificate.c/.h` (DER-encoded certificate + private
key, plus the derived hostname) -- run this **before** building the firmware for a board
that doesn't have it yet; the generated file is checked in like the other `generated/`
outputs but is specific to whichever board last ran the script. For the browser (or OS) to
trust the resulting HTTPS connection without a warning, import
`C:\Users\mail\OneDrive - HSOS\certificates\rootCA.pem.crt` as a trusted root CA once on
the client side.

## <b>Inbetriebnahme / Testen (Commissioning & Testing)</b>

1. `node tools/provision-certificate.mjs` (with the board connected via ST-Link) -- generates
   or reuses that board's HTTPS certificate; see "Certificate provisioning" above. Skip only
   if `Core/Src/generated/device_certificate.c` is already provisioned for this exact board.
2. Insert a formatted SD card (FileX requires this at boot regardless of the web UI, see
   "Assumptions" above).
3. Connect a serial terminal to USART1 (115200 8N1) to see boot/log output.
4. Flash `build/Debug/Nx_WebServer.elf` to the board (ST-Link via STM32CubeProgrammer or
   your IDE's debugger).
5. Connect Ethernet; the board obtains an address via DHCP and prints it to the serial log
   (`IP Address: a.b.c.d`).
6. Import `rootCA.pem.crt` (see "Certificate provisioning") into your browser/OS trust store
   once, then open `https://<board-ip>/` or `https://<hostname>.local/` (the hostname is
   printed in the serial log and is also the certificate's CN, `factory-box-<id>`) -- the
   Modbus Register Console should load and start showing live values within a second
   (status line turns green, "Verbunden -- zuletzt aktualisiert ..."). Flip a valve toggle
   or drag a PWM slider to write a holding register; input registers (sensors, diagnostics)
   are read-only. Without importing the CA, the browser will show a certificate-trust
   warning but the connection still works if you click through it.
7. Confirm mDNS resolution works, e.g. `ping factory-box-<id>.local` from another device on
   the same network segment.
8. Alternatively, exercise the Modbus TCP interface directly with
   `tools/test_modbus.py` (`pip install pymodbus` once):

   ```bash
   python tools/test_modbus.py --host <board-ip>              # dump diagnostics/status
   python tools/test_modbus.py --host <board-ip> --set-valve 1 on
   python tools/test_modbus.py --host <board-ip> --pwm-test    # ramps compressor/conveyor PWM
   ```

9. If USB-PD hardware is attached to the USB-C receptacle, negotiated source capabilities
   and the active voltage/current are logged on the serial console and mirrored into the
   `PWR_PD_*` input registers.

If the web UI never loads or a request appears to hang indefinitely, check the serial log
for a HardFault/UsageFault dump first -- the HTTP server's packet pool is deliberately small
(see "NetX Duo usage hints" above), so a change that grows a single response beyond what
`send_streamed_response()`'s chunking handles is the most likely culprit. If HTTPS itself
fails (handshake errors, connection resets), `TLS_PACKET_BUFFER_SIZE` in `app.cpp` is the
first thing to try increasing -- it's sized conservatively for our small ECDSA (P-256)
server-only certificate and hasn't been validated against every browser/TLS stack yet.
