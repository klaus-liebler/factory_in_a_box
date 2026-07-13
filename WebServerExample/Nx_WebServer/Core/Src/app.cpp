#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <new>
#include "nx_api.h"
#include "fx_api.h"
#include "tx_api.h"
#include "nxd_dhcp_client.h"
#include "nx_web_http_server.h"
#include "nx_stm32_eth_driver.h"
#include "fx_stm32_sd_driver.h"

#include "modbus_tcp_server.hpp"
#include "modbus_register_map.hpp"
#include "diagnostics.hpp"

#include "main.h"

#define NX_CHAR_LITERAL(str) const_cast<CHAR *>(str)

// ADC1-Handle wird in main.c erzeugt (MX_ADC1_Init), hier nur referenziert.
extern "C" ADC_HandleTypeDef hadc1;

// ============================================================================
// Configuration Constants
// ============================================================================

#define TX_APP_MEM_POOL_SIZE        (10 * 1024)
#define FX_APP_MEM_POOL_SIZE        (10 * 1024)
#define NX_APP_MEM_POOL_SIZE        (128 * 1024)

#define DEFAULT_PAYLOAD_SIZE        1536
#define DEFAULT_ARP_CACHE_SIZE      1024
#define DEFAULT_MEMORY_SIZE         1024
#define TOGGLE_LED_PRIORITY         15
#define DEFAULT_PRIORITY            5
#define LINK_PRIORITY               11
#define IO_PRIORITY                 12
#define IO_THREAD_SLEEP_TICKS       50
#define NX_APP_THREAD_PRIORITY      10
#define NX_APP_INSTANCE_PRIORITY    10
#define NX_APP_THREAD_STACK_SIZE    (8 * 1024)
#define Nx_IP_INSTANCE_THREAD_SIZE  (2 * 1024)
#define CONNECTION_PORT             80
#define SERVER_PACKET_SIZE          (2 * NX_WEB_HTTP_SERVER_MIN_PACKET_SIZE)
#define SERVER_STACK                4096
#define SERVER_POOL_SIZE            (SERVER_PACKET_SIZE * 4)
#define NX_APP_PACKET_POOL_SIZE     ((DEFAULT_PAYLOAD_SIZE + sizeof(NX_PACKET)) * 50)
#define NX_APP_DEFAULT_IP_ADDRESS   0
#define NX_APP_DEFAULT_NET_MASK     0

// ============================================================================
// Application State
// ============================================================================

typedef struct {
    NX_IP ip_instance;
    NX_PACKET_POOL packet_pool;
    NX_DHCP dhcp_client;
    NX_WEB_HTTP_SERVER http_server;
    ModbusTcpServer* modbus_server = nullptr;


    FX_MEDIA sd_media;

    ULONG ip_address;
    ULONG net_mask;

    TX_THREAD app_main_thread;
    TX_THREAD led_thread;
    TX_THREAD link_thread;
    TX_THREAD modbus_server_thread;
    TX_THREAD io_thread;
} AppState;

static AppState g_app_state = {0};
static volatile bool blink_led = false;

// ============================================================================
// Success Macro
// ============================================================================

#define ASSURE_SUCCESS(status, message_on_fail) \
if (status != NX_SUCCESS) { \
    printf("Error %s: %s:%d, status: 0x%x\n", message_on_fail, __FILE__, __LINE__, status); \
    Error_Handler(); \
}



// ============================================================================
// Thread Entry Points
// ============================================================================

static void led_thread_entry(ULONG arg);
static void link_thread_entry(ULONG arg);
static void app_main_thread_entry(ULONG arg);
static void io_thread_entry(ULONG arg);

// IP address change callback
static void ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr) {
    (void)ptr;
    if (nx_ip_address_get(ip_instance, &g_app_state.ip_address, &g_app_state.net_mask) != NX_SUCCESS) {
        Error_Handler();
        return;
    }
    if (g_app_state.ip_address != 0) {
        printf("IP Address: %lu.%lu.%lu.%lu\n",
               (g_app_state.ip_address >> 24) & 0xff,
               (g_app_state.ip_address >> 16) & 0xff,
               (g_app_state.ip_address >> 8) & 0xff,
               g_app_state.ip_address & 0xff);
    }
}

// ============================================================================
// Modbus Server Thread
// ============================================================================
static void modbus_server_thread_entry(ULONG thread_input)
{
    ModbusTcpServer* server = (ModbusTcpServer*)thread_input;
    if (server) {
        printf("Modbus TCP Server started on port 502\n");
        server->run();  // Infinite loop, akzeptiert Clients
        printf("Modbus TCP Server stopped\n");
    }
}

// ============================================================================
// LED Thread
// ============================================================================

static void led_thread_entry(ULONG arg) {
    (void)arg;
    while (1) {
        if (blink_led) {
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        }else{
            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
        }
        tx_thread_sleep(50);
    }
}

// ============================================================================
// Link Thread (Ethernet cable detection)
// ============================================================================

static void link_thread_entry(ULONG arg) {
    (void)arg;
    ULONG actual_status;
    int linkdown = 0;

    while (1) {
        UINT status = nx_ip_interface_status_check(&g_app_state.ip_instance, 0,
                                                    NX_IP_LINK_ENABLED,
                                                    &actual_status, 10);

        if (status == NX_SUCCESS) {
            if (linkdown == 1) {
                linkdown = 0;
                printf("Network cable connected\n");
                nx_ip_driver_direct_command(&g_app_state.ip_instance, NX_LINK_ENABLE, &actual_status);

                status = nx_ip_interface_status_check(&g_app_state.ip_instance, 0,
                                                      NX_IP_ADDRESS_RESOLVED,
                                                      &actual_status, 10);
                if (status == NX_SUCCESS) {
                    nx_dhcp_stop(&g_app_state.dhcp_client);
                    nx_dhcp_reinitialize(&g_app_state.dhcp_client);
                    nx_dhcp_start(&g_app_state.dhcp_client);
                }
            }
        } else {
            if (linkdown == 0) {
                linkdown = 1;
                printf("Network cable disconnected\n");
                nx_ip_driver_direct_command(&g_app_state.ip_instance, NX_LINK_DISABLE, &actual_status);
            }
        }

        tx_thread_sleep(NX_IP_PERIODIC_RATE);
    }
}

// ============================================================================
// I/O Thread (digitale Eingaenge, Drucksensor, Ventile, Diagnostik) -- Stage 1
// ============================================================================

static void io_thread_entry(ULONG arg) {
    (void)arg;

    while (1) {
        if (g_app_state.modbus_server) {
            ModbusTcpServer& server = *g_app_state.modbus_server;

            // Lichtschranken (NPN, idle-high -> aktiv = LOW). LS3/PC8 auf dem
            // Eval-Board nicht verdrahtet (Konflikt mit SDMMC1/FileX, s. Plan).
            bool ls1_active = (HAL_GPIO_ReadPin(LIGHTBARRIER1_GPIO_Port, LIGHTBARRIER1_Pin) == GPIO_PIN_RESET);
            bool ls2_active = (HAL_GPIO_ReadPin(LIGHTBARRIER2_GPIO_Port, LIGHTBARRIER2_Pin) == GPIO_PIN_RESET);
            server.write_input_register(ModbusRegisters::Input::LIGHTBARRIER1, ls1_active ? 1 : 0);
            server.write_input_register(ModbusRegisters::Input::LIGHTBARRIER2, ls2_active ? 1 : 0);

            // Analoger Drucksensor -- Rohwert, physikalische Skalierung noch offen
            if (HAL_ADC_Start(&hadc1) == HAL_OK) {
                if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
                    uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
                    server.write_input_register(ModbusRegisters::Input::PRESSURE_RAW, raw);
                }
                HAL_ADC_Stop(&hadc1);
            }

            // Pneumatikventile aus Holding-Registern setzen
            bool valve1 = server.read_holding_register(ModbusRegisters::Holding::VALVE1) != 0;
            bool valve2 = server.read_holding_register(ModbusRegisters::Holding::VALVE2) != 0;
            bool valve3 = server.read_holding_register(ModbusRegisters::Holding::VALVE3) != 0;
            HAL_GPIO_WritePin(VALVE1_GPIO_Port, VALVE1_Pin, valve1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(VALVE2_GPIO_Port, VALVE2_Pin, valve2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(VALVE3_GPIO_Port, VALVE3_Pin, valve3 ? GPIO_PIN_SET : GPIO_PIN_RESET);

            // Ethernet-Link-Status + HealthState-Aggregation
            ULONG actual_status;
            bool eth_link_up = (nx_ip_interface_status_check(&g_app_state.ip_instance, 0,
                                                              NX_IP_LINK_ENABLED, &actual_status, 10) == NX_SUCCESS);
            server.write_input_register(ModbusRegisters::Input::ETH_LINK_STATUS, eth_link_up ? 1 : 0);
            Diagnostics::update_health_state(server, eth_link_up);
        }

        tx_thread_sleep(IO_THREAD_SLEEP_TICKS);
    }
}

// ============================================================================
// Main App Thread (DHCP/startup)
// ============================================================================

static void app_main_thread_entry(ULONG arg) {
    (void)arg;

    uint32_t data_buffer[512];

    ASSURE_SUCCESS(fx_media_open(&g_app_state.sd_media, NX_CHAR_LITERAL("STM32_SDIO_DISK"),
                           fx_stm32_sd_driver, 0,
                           (VOID *)data_buffer, sizeof(data_buffer)),
                           "FileX media open failed");

    printf("FileX media opened\n");

    NX_WEB_HTTP_SERVER_MIME_MAP mime_maps[] = {
        {NX_CHAR_LITERAL("css"), NX_CHAR_LITERAL("text/css")},
        {NX_CHAR_LITERAL("svg"), NX_CHAR_LITERAL("image/svg+xml")},
        {NX_CHAR_LITERAL("png"), NX_CHAR_LITERAL("image/png")},
        {NX_CHAR_LITERAL("jpg"), NX_CHAR_LITERAL("image/jpg")}
    };
    nx_web_http_server_mime_maps_additional_set(&g_app_state.http_server, mime_maps, 4);

    ASSURE_SUCCESS(nx_web_http_server_start(&g_app_state.http_server), "HTTP Server start failed");
    printf("HTTP Server started\n");


    ASSURE_SUCCESS(nx_ip_address_change_notify(&g_app_state.ip_instance,
                                      ip_address_change_notify_callback,
                                      NULL), "IP address change notify failed");

    ASSURE_SUCCESS(nx_dhcp_start(&g_app_state.dhcp_client), "DHCP start failed");
}

// ============================================================================
// HTTP Request Callback
// ============================================================================

static UINT webserver_request_callback(NX_WEB_HTTP_SERVER *server_ptr, UINT request_type,
                                       CHAR *resource, NX_PACKET *packet_ptr) {
    (void)request_type;
    (void)packet_ptr;

    char response_data[512] = {0};
    char response_type[30] = {0};

    if (strcmp(resource, "/GetNetInfo") == 0) {
        snprintf(response_data, sizeof(response_data), "%lu.%lu.%lu.%lu,80",
                 (g_app_state.ip_address >> 24) & 0xff,
                 (g_app_state.ip_address >> 16) & 0xff,
                 (g_app_state.ip_address >> 8) & 0xff,
                 g_app_state.ip_address & 0xff);
    } else if (strcmp(resource, "/LedOn") == 0) {
        printf("LED On\n");
        blink_led = true;
        sprintf(response_data, "OK");
    } else if (strcmp(resource, "/LedOff") == 0) {
        printf("LED Off\n");
        blink_led = false;
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
        sprintf(response_data, "OK");
    } else {
        return NX_SUCCESS;
    }

    UINT string_length;
    nx_web_http_server_type_get(server_ptr, resource, response_type, &string_length);
    response_type[string_length] = '\0';

    NX_PACKET *resp_packet;
    if (nx_web_http_server_callback_generate_response_header(
            server_ptr, &resp_packet, NX_CHAR_LITERAL(NX_WEB_HTTP_STATUS_OK),
            strlen(response_data), response_type, NX_NULL) != NX_SUCCESS) {
        return NX_NOT_SUCCESSFUL;
    }

    if (nx_packet_data_append(resp_packet, response_data, strlen(response_data),
                              server_ptr->nx_web_http_server_packet_pool_ptr,
                              NX_WAIT_FOREVER) != NX_SUCCESS) {
        nx_packet_release(resp_packet);
        return NX_NOT_SUCCESSFUL;
    }

    if (nx_web_http_server_callback_packet_send(server_ptr, resp_packet) != NX_SUCCESS) {
        nx_packet_release(resp_packet);
        return NX_NOT_SUCCESSFUL;
    }

    return NX_WEB_HTTP_CALLBACK_COMPLETED;
}


// ============================================================================
// ThreadX Application Entry Point
// ============================================================================
extern "C" void tx_application_define(void *first_unused_memory) {
    printf("001 Application initialization starting...\n");

    static UCHAR tx_byte_pool_buffer[TX_APP_MEM_POOL_SIZE] __attribute__((aligned(4)));
    static TX_BYTE_POOL tx_app_byte_pool;

    static UCHAR fx_byte_pool_buffer[FX_APP_MEM_POOL_SIZE] __attribute__((aligned(4)));
    static TX_BYTE_POOL fx_app_byte_pool;

    static UCHAR nx_byte_pool_buffer[NX_APP_MEM_POOL_SIZE] __attribute__((aligned(4)));
    static TX_BYTE_POOL nx_app_byte_pool;

    ASSURE_SUCCESS(tx_byte_pool_create(&tx_app_byte_pool, NX_CHAR_LITERAL("Tx App Pool"),
                                 tx_byte_pool_buffer, TX_APP_MEM_POOL_SIZE), "Tx App Pool create failed");


    ASSURE_SUCCESS(tx_byte_pool_create(&fx_app_byte_pool, NX_CHAR_LITERAL("Fx App Pool"),
                                 fx_byte_pool_buffer, FX_APP_MEM_POOL_SIZE), "Fx App Pool create failed");

    ASSURE_SUCCESS(tx_byte_pool_create(&nx_app_byte_pool, NX_CHAR_LITERAL("Nx App Pool"),
                                 nx_byte_pool_buffer, NX_APP_MEM_POOL_SIZE), "Nx App Pool create failed");

    fx_system_initialize();
    nx_system_initialize();

    void *ptr=0;
    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, NX_APP_PACKET_POOL_SIZE, TX_NO_WAIT), "NetXDuo App Pool allocate failed");
    ASSURE_SUCCESS(nx_packet_pool_create(&g_app_state.packet_pool, NX_CHAR_LITERAL("NetXDuo App Pool"),
                                DEFAULT_PAYLOAD_SIZE, ptr, NX_APP_PACKET_POOL_SIZE), "NetXDuo App Pool create failed");

    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, Nx_IP_INSTANCE_THREAD_SIZE, TX_NO_WAIT), "IP instance memory allocate failed");
    ASSURE_SUCCESS(nx_ip_create(&g_app_state.ip_instance, NX_CHAR_LITERAL("NetX IP"),
                       NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK,
                       &g_app_state.packet_pool, nx_stm32_eth_driver,
                       (UCHAR *)ptr, Nx_IP_INSTANCE_THREAD_SIZE,
                       NX_APP_INSTANCE_PRIORITY), "IP create failed");

    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, DEFAULT_ARP_CACHE_SIZE, TX_NO_WAIT), "ARP cache allocate failed");
    ASSURE_SUCCESS(nx_arp_enable(&g_app_state.ip_instance, (VOID *)ptr, DEFAULT_ARP_CACHE_SIZE), "ARP enable failed");

    ASSURE_SUCCESS(nx_icmp_enable(&g_app_state.ip_instance), "ICMP enable failed");

    ASSURE_SUCCESS(nx_tcp_enable(&g_app_state.ip_instance), "TCP enable failed");

    ASSURE_SUCCESS(nx_udp_enable(&g_app_state.ip_instance), "UDP enable failed");

    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, NX_APP_THREAD_STACK_SIZE, TX_NO_WAIT), "App Main Thread stack allocate failed");
    tx_thread_create(&g_app_state.app_main_thread, NX_CHAR_LITERAL("App Main Thread"),
                     app_main_thread_entry, 0,
                     (VOID *)ptr, NX_APP_THREAD_STACK_SIZE,
                     NX_APP_THREAD_PRIORITY, NX_APP_THREAD_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, DEFAULT_MEMORY_SIZE, TX_NO_WAIT), "LED Thread stack allocate failed");
    tx_thread_create(&g_app_state.led_thread, NX_CHAR_LITERAL("LED Thread"),
                     led_thread_entry, 0,
                     (VOID *)ptr, DEFAULT_MEMORY_SIZE,
                     TOGGLE_LED_PRIORITY, TOGGLE_LED_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, 2 * DEFAULT_MEMORY_SIZE, TX_NO_WAIT), "Link Thread stack allocate failed");
    tx_thread_create(&g_app_state.link_thread, NX_CHAR_LITERAL("Link Thread"),
                     link_thread_entry, 0,
                     (VOID *)ptr, 2 * DEFAULT_MEMORY_SIZE,
                     LINK_PRIORITY, LINK_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, SERVER_POOL_SIZE, TX_NO_WAIT), "HTTP Server Pool allocate failed");
    NX_PACKET_POOL *server_pool = (NX_PACKET_POOL *)ptr;
    ASSURE_SUCCESS(nx_packet_pool_create(server_pool, NX_CHAR_LITERAL("HTTP Server Pool"),
                                SERVER_PACKET_SIZE, (VOID *)(server_pool + 1),
                                SERVER_POOL_SIZE - sizeof(NX_PACKET_POOL)), "HTTP Server Pool create failed");

    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, SERVER_STACK, TX_NO_WAIT), "HTTP Server stack allocate failed");
    ASSURE_SUCCESS(nx_web_http_server_create(&g_app_state.http_server, NX_CHAR_LITERAL("HTTP Server"),
                                    &g_app_state.ip_instance, CONNECTION_PORT,
                                    &g_app_state.sd_media, (VOID *)ptr, SERVER_STACK,
                                    server_pool, NX_NULL,
                                    webserver_request_callback), "HTTP Server create failed");

    ASSURE_SUCCESS(nx_dhcp_create(&g_app_state.dhcp_client, &g_app_state.ip_instance, NX_CHAR_LITERAL("DHCP Client")), "DHCP Client create failed");

        // --- Modbus TCP Server Setup ---
    // Allocate memory for Modbus server instance

    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, sizeof(ModbusTcpServer), TX_NO_WAIT), "Modbus server allocate failed");
    g_app_state.modbus_server = new(ptr) ModbusTcpServer(&g_app_state.ip_instance, &g_app_state.packet_pool);

    // Initialisiere Modbus Server
    ASSURE_SUCCESS(g_app_state.modbus_server->initialize(), "Modbus TCP Server initialization failed");

    // Diagnostik-Register (Chip-ID, Firmware-Version) einmalig befuellen
    Diagnostics::write_startup_registers(*g_app_state.modbus_server);

    // Allocate the Modbus server thread stack
    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, 2 * DEFAULT_MEMORY_SIZE, TX_NO_WAIT), "Modbus server thread stack allocate failed");

    // Create the Modbus server thread (but don't start yet)
    ASSURE_SUCCESS(tx_thread_create(
        &g_app_state.modbus_server_thread,
        NX_CHAR_LITERAL("Modbus Server Thread"),
        modbus_server_thread_entry,
        (ULONG)g_app_state.modbus_server,  // thread_input = pointer to server
        ptr,
        2 * DEFAULT_MEMORY_SIZE,
        DEFAULT_PRIORITY,
        DEFAULT_PRIORITY,
        TX_NO_TIME_SLICE,
        TX_AUTO_START
    ), "Modbus server thread create failed");

    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, 2 * DEFAULT_MEMORY_SIZE, TX_NO_WAIT), "IO Thread stack allocate failed");
    tx_thread_create(&g_app_state.io_thread, NX_CHAR_LITERAL("IO Thread"),
                     io_thread_entry, 0,
                     (VOID *)ptr, 2 * DEFAULT_MEMORY_SIZE,
                     IO_PRIORITY, IO_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    printf("Application initialization complete\n");
}
