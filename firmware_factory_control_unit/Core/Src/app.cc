#include "app.hh"
#include "modbus_register_model.hh"
#include "tx_api.h"
#include "net_setup.hpp"
#include "gitconstants.hh"
#include "generated/device_hostname.hh"

extern "C" ADC_HandleTypeDef hadc1;

constexpr uint32_t DEFAULT_MEMORY_SIZE = 1024;
constexpr uint32_t DEFAULT_PRIORITY = 5;
constexpr uint32_t NX_APP_THREAD_STACK_SIZE = 8 * 1024;
constexpr uint32_t NX_APP_THREAD_PRIORITY = 10;
constexpr uint32_t IO_THREAD_STACK_SIZE = 6 * 1024;
// War 12 (niedrigere Prioritaet als NetX' eigener IP-Thread, s. net_setup.cpp
// NX_APP_INSTANCE_PRIORITY=10) -- ThreadX: kleinere Zahl = hoehere Prioritaet. Der IP-Thread
// verarbeitet ALLE eingehenden Pakete (TCP/HTTP/DHCP/mDNS) und verdraengte den IO-Thread damit
// bei jedem Webseiten-Reload komplett auf Thread-Ebene (nicht nur kurz per Interrupt), teils
// hunderte ms am Stueck -- das liess I2C4-/SPI2-Transfers (INA226, HX711) mit Timeout
// fehlschlagen, selbst nachdem ETH_IRQn/ETH_WKUP_IRQn (NVIC-Ebene, s. stm32h5xx_hal_msp.c)
// bereits abgesenkt wurden. Jetzt hoeher als der NetX-IP-Thread (10) und die App-Threads: der
// IO-Thread schlaeft ohnehin ~50ms pro Zyklus (s. io.hpp IO_THREAD_SLEEP_TICKS), gibt dem
// Netzwerk-Stack also reichlich Gelegenheit, ohne dass der Sensor-Zyklus verdraengt wird.
constexpr uint32_t IO_THREAD_PRIORITY = 8;
constexpr uint32_t USBD_DEVICE_THREAD_STACK_SIZE = 4 * 1024;
// Zwischen Modbus (5, zeitkritischste TCP-Antworten) und io_thread (8) angesiedelt -- s.
// Begruendung in app.hh (USB-Full-Speed-Timing vertraegt keinen 50ms-Zyklus).
constexpr uint32_t USBD_DEVICE_THREAD_PRIORITY = 6;
constexpr uint32_t HEARTBEAT_THREAD_STACK_SIZE = 2 * 1024;
// Niedrigste Prioritaet aller App-Threads (grosse Zahl) -- reine Diagnose, darf nirgendwo
// dazwischenfunken, schlaeft ohnehin 3s pro Zyklus.
constexpr uint32_t HEARTBEAT_THREAD_PRIORITY = 12;

// TX_MUTEX{} zero-initialisiert u.a. tx_mutex_id -- tx_mutex_create() setzt dieses Feld
// intern auf die magische Konstante TX_MUTEX_ID (ungleich 0, s. tx_mutex.h), daher genuegt ein
// Test auf tx_mutex_id!=0 als "wurde bereits erzeugt"-Pruefung, ohne ein zusaetzliches
// bool-Flag zu pflegen. Schuetzt __malloc_lock/__malloc_unlock vor Aufrufen, die vor
// tx_application_define() passieren koennen (z.B. malloc() aus globalen Konstruktoren).
static TX_MUTEX malloc_mutex{};
static TX_MUTEX log_mutex{};

static void log_lock(bool lock) {
    if (lock) {
        tx_mutex_get(&log_mutex, TX_WAIT_FOREVER);
    } else {
        tx_mutex_put(&log_mutex);
    }
}

extern "C" void __malloc_lock(struct _reent *reent){
    (void)reent;
    if (malloc_mutex.tx_mutex_id == 0) return;
    tx_mutex_get(&malloc_mutex, TX_WAIT_FOREVER);
}

extern "C" void __malloc_unlock(struct _reent *reent){
    (void)reent;
    if (malloc_mutex.tx_mutex_id == 0) return;
    tx_mutex_put(&malloc_mutex);
}

extern "C" void AppSetupBeforeThreadX() {
    App::Instance().SetupBeforeThreadX();
}

extern "C" void tx_application_define(void *first_unused_memory) {
    (void)first_unused_memory;
    auto& app = App::Instance();

    tx_mutex_create(&malloc_mutex, const_cast<CHAR *>("malloc_mutex"), TX_INHERIT);
    tx_mutex_create(&log_mutex, const_cast<CHAR *>("Log Mutex"), TX_NO_INHERIT);
    log_set_lock(log_lock);
    log_set_level(LOG_INFO);

    app.register_model->ArmForMultithreadingWithMutex();

    log_info("tx_application_define() called, first_unused_memory=%p", first_unused_memory);

    static UCHAR byte_pool_buffer[NX_APP_MEM_POOL_SIZE] __attribute__((aligned(4)));
    XASSERT(tx_byte_pool_create(&app.byte_pool, _C("Byte Pool"),
                                 byte_pool_buffer, NX_APP_MEM_POOL_SIZE), "Byte Pool create failed");
    fx_system_initialize();
    nx_system_initialize();
    net_setup_create(&app, &app.byte_pool);

    void *ptr = nullptr;
    XASSERT(tx_byte_allocate(&app.byte_pool, &ptr, NX_APP_THREAD_STACK_SIZE, TX_NO_WAIT), "App Main Thread stack allocate failed");
    XASSERT(tx_thread_create(&app.app_main_thread, _C("App Main Thread"),
                     App::AppThreadStatic, (ULONG)&app,
                     ptr, NX_APP_THREAD_STACK_SIZE,
                     NX_APP_THREAD_PRIORITY, NX_APP_THREAD_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START), "App Main Thread create failed");
    log_info("Application initialization complete");
}

void App::AppThreadStatic(ULONG arg) {
    reinterpret_cast<App*>(arg)->AppThread();
}

void App::ModbusServerThreadStatic(ULONG arg) {
    reinterpret_cast<App*>(arg)->ModbusServerThread();
}

void App::IOThreadStatic(ULONG arg) {
    reinterpret_cast<App*>(arg)->IOThread();
}

void App::UsbdDeviceThreadStatic(ULONG arg) {
    reinterpret_cast<App*>(arg)->UsbdDeviceThread();
}

void App::HeartbeatThreadStatic(ULONG arg) {
    reinterpret_cast<App*>(arg)->HeartbeatThread();
}

void App::ModbusServerThread(){
    this->modbus_server->initialize();
    log_info("Modbus TCP Server started on port 502");
    this->modbus_server->run();
}

void App::IOThread() {
    this->io->Setup();
    log_info("I/O Thread started");
    this->io->Loop();
}

[[noreturn]] void App::UsbdDeviceThread() {
    uint8_t ncm_mac[6];
    App::ComputeNcmMac(this->chip_uid, ncm_mac);
    usbd_device_setup(this->chip_uid[0], this->chip_uid[1], this->chip_uid[2], ncm_mac);
    log_info("USB Device Thread started");
    usbd_device_loop(App::UsbdDevicePollHook);
}

// cdc_itf=1: zweite CDC-Deskriptorinstanz in usbd_device.c (ITF_CDC1_MODBUS_*), Instanz 0 ist
// "FactoryControl Debug". slave_id=1 (Modbus-Standard-Default, ueber Modbus-Register aktuell
// nicht konfigurierbar -- bei Bedarf spaeter aus einem Holding-Register lesen).
void App::UsbdDevicePollHook() {
    App::Instance().modbus_rtu_server->Poll();
    usb_ncm_driver_poll();
}

void App::ComputeNcmMac(const uint32_t chip_uid[3], uint8_t mac[6]) {
    mac[0] = 0x02;
    mac[1] = (uint8_t)(chip_uid[0] >> 24);
    mac[2] = (uint8_t)(chip_uid[0]);
    mac[3] = (uint8_t)(chip_uid[1] >> 24);
    mac[4] = (uint8_t)(chip_uid[1]);
    mac[5] = (uint8_t)(chip_uid[2]);
}

// Rein diagnostisch: liest ausschliesslich bereits vom io_thread periodisch aktualisierte
// Register (CPU_TEMPERATURE_C/FREE_HEAP_KIB/PWR_*, s. cpu_temp.hh/power.hh/io.cpp) -- kein
// eigener Zugriff auf DTS/I2C4, daher unkritisch bzgl. Peripherie-Besitz/Timing des io_threads.
// PWR_STATUS==0 bedeutet "INA226 erkannt und Read() erfolgreich" (s. power.hh); ohne das faellt
// die Zeile auf einen "n/a"-Hinweis zurueck statt eine Spannung/Strom von 0 vorzutaeuschen.
[[noreturn]] void App::HeartbeatThread() {
    log_info("Heartbeat Thread started");
    while (true) {
        int16_t cpu_temp_c = (int16_t)this->register_model->GetInputRegister(ModbusRegisters::Input::CPU_TEMPERATURE_C);
        uint16_t free_heap_kib = this->register_model->GetInputRegister(ModbusRegisters::Input::FREE_HEAP_KIB);
        bool power_ok = this->register_model->GetInputRegister(ModbusRegisters::Input::PWR_STATUS) == 0;
        ULONG tx_ticks_now = tx_time_get();

        if (power_ok) {
            uint16_t bus_mv = this->register_model->GetInputRegister(ModbusRegisters::Input::PWR_BUS_VOLTAGE_MV);
            int16_t current_ma = (int16_t)this->register_model->GetInputRegister(ModbusRegisters::Input::PWR_CURRENT_MA);
            log_info("Heartbeat: CPU=%d C, FreeHeap=%u KiB, Bus=%u mV, Current=%d mA",
                     (int)cpu_temp_c, (unsigned)free_heap_kib, (unsigned)bus_mv, (int)current_ma);
        } else {
            log_info("Heartbeat: CPU=%d C, FreeHeap=%u KiB, Power=n/a (INA226 nicht erkannt)",
                     (int)cpu_temp_c, (unsigned)free_heap_kib);
        }

        // Bis zur naechsten durch 3 teilbaren Sekunde schlafen (statt starr 3s ab jetzt) --
        // Ausgaben landen dadurch auf einem festen 0/3/6/9...-Sekundenraster seit Boot, statt
        // mit der Zeit zu "wandern" (die Laufzeit des Schleifenkoerpers selbst wuerde sich sonst
        // von Durchlauf zu Durchlauf aufsummieren).
        ULONG now_seconds = tx_ticks_now / TX_TIMER_TICKS_PER_SECOND;
        ULONG next_seconds = ((now_seconds / 3) + 1) * 3;
        ULONG target_ticks = next_seconds * TX_TIMER_TICKS_PER_SECOND;
        // -1: tx_thread_sleep(N) garantiert nur "mindestens N Ticks" -- in der Praxis wacht der
        // Thread erst nach N+1 Ticks auf (er wartet den aktuell schon angebrochenen Tick noch
        // vollstaendig ab, s. Debugging-Sitzung: txTicks landete deshalb immer auf .../...01
        // statt .../...00). Ein Tick weniger anfordern kompensiert das, sodass wir tatsaechlich
        // exakt auf target_ticks aufwachen. sleep_ticks bleibt dabei immer >=1 (target liegt per
        // Konstruktion mindestens 1 Tick, hoechstens 300 Ticks in der Zukunft).
        ULONG sleep_ticks = target_ticks - tx_ticks_now - 1;
        tx_thread_sleep(sleep_ticks);
    }
}

void App::AppThread() {
    void *ptr = nullptr;

    // Muss die allererste Zeile sein: ab hier laeuft echter ThreadX-Thread-Kontext (dieser
    // Thread ist der einzige mit TX_AUTO_START), tx_thread_sleep() ist somit ab jetzt ueberall
    // im Programm zulaessig -- s. hal_tick_threadx.c/HAL_Delay().
    hal_tick_threadx_mark_running();

    net_setup_start(this);

    this->modbus_server = new ModbusTcpServer(&this->ip_instance, &this->packet_pool, *this->register_model);

    // Braucht keinen eigenen Thread (im Gegensatz zu ModbusTcpServer::run(), die blockierend auf
    // einen TCP-Accept wartet): Poll() wird nicht-blockierend aus dem usbd_device_thread heraus
    // angestossen (s. App::UsbdDevicePollHook()).
    this->modbus_rtu_server = new ModbusRtuServer(1, *this->register_model);

    XASSERT(tx_byte_allocate(&this->byte_pool, &ptr, 2 * DEFAULT_MEMORY_SIZE, TX_NO_WAIT), "Modbus server thread stack allocate failed");
    XASSERT(tx_thread_create(
        &this->modbus_server_thread,
        _C("Modbus Server Thread"),
        ModbusServerThreadStatic,
        (ULONG)this,
        ptr,
        2 * DEFAULT_MEMORY_SIZE,
        DEFAULT_PRIORITY,
        DEFAULT_PRIORITY,
        TX_NO_TIME_SLICE,
        TX_AUTO_START
    ), "Modbus server thread create failed");

    this->io = new Io(*this->register_model, this->ip_instance, this->dhcp_client, *this->usb_pd_control);
    // io->Setup() (registerlevel-Init aller Subsysteme: ADC-Start, PWM-Modus-Fixups, TMC2209-
    // UART-Init, I2C-Sensor-Init, FDCAN-Filter, WS2812/LED-Init) laeuft im IO-Thread selbst
    // (App::IOThread()), NICHT hier -- mehrere der darin aufgerufenen Setup()-Funktionen
    // brauchen funktionierende HAL-Timeouts (HAL_Delay()/HAL_I2C_*), die waehrend
    // tx_application_define() nicht funktionieren (Interrupts gesperrt, solange
    // tx_kernel_enter() laeuft).
    XASSERT(tx_byte_allocate(&this->byte_pool, &ptr, IO_THREAD_STACK_SIZE, TX_NO_WAIT), "IO Thread stack allocate failed");
    XASSERT(tx_thread_create(&this->io_thread, _C("IO Thread"),
                     App::IOThreadStatic, (ULONG)this,
                     ptr, IO_THREAD_STACK_SIZE,
                     IO_THREAD_PRIORITY, IO_THREAD_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START), "IO thread create failed");

    // Reaktionsschnell (s. Begruendung in app.hh): niedrigere Prioritaetszahl = hoehere
    // Prioritaet als io_thread, damit USB-Enumeration/Bulk-Transfers nicht durch den
    // 50ms-Sensor-/Aktor-Zyklus ausgebremst werden.
    XASSERT(tx_byte_allocate(&this->byte_pool, &ptr, USBD_DEVICE_THREAD_STACK_SIZE, TX_NO_WAIT), "USB Device Thread stack allocate failed");
    XASSERT(tx_thread_create(&this->usbd_device_thread, _C("USB Device Thread"),
                     App::UsbdDeviceThreadStatic, (ULONG)this,
                     ptr, USBD_DEVICE_THREAD_STACK_SIZE,
                     USBD_DEVICE_THREAD_PRIORITY, USBD_DEVICE_THREAD_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START), "USB Device thread create failed");

    XASSERT(tx_byte_allocate(&this->byte_pool, &ptr, HEARTBEAT_THREAD_STACK_SIZE, TX_NO_WAIT), "Heartbeat Thread stack allocate failed");
    XASSERT(tx_thread_create(&this->heartbeat_thread, _C("Heartbeat Thread"),
                     App::HeartbeatThreadStatic, (ULONG)this,
                     ptr, HEARTBEAT_THREAD_STACK_SIZE,
                     HEARTBEAT_THREAD_PRIORITY, HEARTBEAT_THREAD_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START), "Heartbeat thread create failed");
}

void App::fillRegistersWithInitialValues() {
    this->register_model->SetInputRegister(ModbusRegisters::Input::FW_VERSION_MAJOR, FW_VERSION_MAJOR);
    this->register_model->SetInputRegister(ModbusRegisters::Input::FW_VERSION_MINOR, FW_VERSION_MINOR);
    this->register_model->SetInputRegister(ModbusRegisters::Input::CHIP_ID_W0_HI, (uint16_t)(chip_uid[0] >> 16));
    this->register_model->SetInputRegister(ModbusRegisters::Input::CHIP_ID_W0_LO, (uint16_t)(chip_uid[0] & 0xFFFF));
    this->register_model->SetInputRegister(ModbusRegisters::Input::CHIP_ID_W1_HI, (uint16_t)(chip_uid[1] >> 16));
    this->register_model->SetInputRegister(ModbusRegisters::Input::CHIP_ID_W1_LO, (uint16_t)(chip_uid[1] & 0xFFFF));
    this->register_model->SetInputRegister(ModbusRegisters::Input::CHIP_ID_W2_HI, (uint16_t)(chip_uid[2] >> 16));
    this->register_model->SetInputRegister(ModbusRegisters::Input::CHIP_ID_W2_LO, (uint16_t)(chip_uid[2] & 0xFFFF));
}

void App::greeting() {
    log_info("");
    log_info("=== FactoryInABox Control Unit starting ===");
    log_info("Board:    %.*s", (int)BOARD_NAME.length(), BOARD_NAME.data());
    log_info("Hostname: %s.local", DEVICE_HOSTNAME);
    log_info("--- Build Information ---");
    log_info("  Version: %.*s", (int)git::VERSION.length(), git::VERSION.data());
    log_info("  Commit:  %.*s (%.*s)", (int)git::COMMIT_HASH.length(), git::COMMIT_HASH.data(),
            (int)git::BRANCH.length(), git::BRANCH.data());
    log_info("  Message: %.*s", (int)git::COMMIT_MESSAGE.length(), git::COMMIT_MESSAGE.data());
    log_info("  Author:  %.*s", (int)git::COMMIT_AUTHOR.length(), git::COMMIT_AUTHOR.data());
    log_info("  Built:   %.*s", (int)git::BUILD_TIMESTAMP.length(), git::BUILD_TIMESTAMP.data());
    if (git::IS_DIRTY) {
        log_warn("Working directory has uncommitted changes!");
    } else {
        log_info("Working directory was clean relative to Git.");
    }

    log_info("--- Clocks ---");
    log_info("  SYSCLK: %lu MHz", (unsigned long)(HAL_RCC_GetSysClockFreq() / 1000000));
    log_info("  HCLK:   %lu MHz", (unsigned long)(HAL_RCC_GetHCLKFreq() / 1000000));
    log_info("  PCLK1:  %lu MHz", (unsigned long)(HAL_RCC_GetPCLK1Freq() / 1000000));
    log_info("  PCLK2:  %lu MHz", (unsigned long)(HAL_RCC_GetPCLK2Freq() / 1000000));

    // UID_BASE liegt im OTP-/System-Speicherbereich, nicht im regulaeren Flash-Adressraum, den
    // der ICACHE ueberwacht -- ein Read bei aktivem ICACHE erzeugt einen precise BusFault.
    HAL_ICACHE_Disable();
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();
    HAL_ICACHE_Enable();
    log_info("--- Chip ---");
    log_info("  UID: %08lX-%08lX-%08lX", (unsigned long)uid0, (unsigned long)uid1, (unsigned long)uid2);
    log_info("  Reset cause: %s", reset_cause());
    __HAL_RCC_CLEAR_RESET_FLAGS();

    log_info("  MAC: %02X:%02X:%02X:%02X:%02X:%02X",
            heth.Init.MACAddr[0], heth.Init.MACAddr[1], heth.Init.MACAddr[2],
            heth.Init.MACAddr[3], heth.Init.MACAddr[4], heth.Init.MACAddr[5]);

    log_info("============================================");
}

void App::SetupBeforeThreadX() {
    this->register_model = BuildModbusRegisterModel();

    // Ganz zuerst: falls ein USB-PD-Netzteil die Versorgung uebernimmt:
    // Blockieren, bis Spannung=20V
    this->usb_pd_control = new USBPDControl(this->register_model);
    this->usb_pd_control->EarlySetup(20000);

    HAL_ICACHE_Disable();
    this->chip_uid[0] = HAL_GetUIDw0();
    this->chip_uid[1] = HAL_GetUIDw1();
    this->chip_uid[2] = HAL_GetUIDw2();
    HAL_ICACHE_Enable();

    fillRegistersWithInitialValues();
    greeting();

    // Reset-Pin des LAN8720-Phy kurz auf LOW, damit er sauber startet (s. LAN8720-Datenblatt).
    HAL_GPIO_WritePin(ETH_RESET_GPIO_Port, ETH_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(ETH_RESET_GPIO_Port, ETH_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    // ADC1 Continuous-Conversion-Modus starten
    HAL_ADC_Start(&hadc1);
}
