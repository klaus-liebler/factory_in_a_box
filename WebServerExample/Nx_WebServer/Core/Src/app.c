#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "nx_api.h"
#include "fx_api.h"
#include "tx_api.h"
#include "nxd_dhcp_client.h"
#include "nx_web_http_server.h"
#include "nx_stm32_eth_driver.h"
#include "fx_stm32_sd_driver.h"
#include "main.h"

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
    FX_MEDIA sd_media;

    ULONG ip_address;
    ULONG net_mask;

    TX_THREAD app_main_thread;
    TX_THREAD led_thread;
    TX_THREAD link_thread;
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
// Main App Thread (DHCP/startup)
// ============================================================================

static void app_main_thread_entry(ULONG arg) {
    (void)arg;

    uint32_t data_buffer[512];

    ASSURE_SUCCESS(fx_media_open(&g_app_state.sd_media, "STM32_SDIO_DISK",
                           fx_stm32_sd_driver, 0,
                           (VOID *)data_buffer, sizeof(data_buffer)),
                           "FileX media open failed");

    printf("FileX media opened\n");

    NX_WEB_HTTP_SERVER_MIME_MAP mime_maps[] = {
        {"css", "text/css"},
        {"svg", "image/svg+xml"},
        {"png", "image/png"},
        {"jpg", "image/jpg"}
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
            server_ptr, &resp_packet, NX_WEB_HTTP_STATUS_OK,
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
extern void tx_application_define(void *first_unused_memory) {
    printf("001 Application initialization starting...\n");

    static UCHAR tx_byte_pool_buffer[TX_APP_MEM_POOL_SIZE] __attribute__((aligned(4)));
    static TX_BYTE_POOL tx_app_byte_pool;

    static UCHAR fx_byte_pool_buffer[FX_APP_MEM_POOL_SIZE] __attribute__((aligned(4)));
    static TX_BYTE_POOL fx_app_byte_pool;

    static UCHAR nx_byte_pool_buffer[NX_APP_MEM_POOL_SIZE] __attribute__((aligned(4)));
    static TX_BYTE_POOL nx_app_byte_pool;

    ASSURE_SUCCESS(tx_byte_pool_create(&tx_app_byte_pool, "Tx App Pool",
                                 tx_byte_pool_buffer, TX_APP_MEM_POOL_SIZE), "Tx App Pool create failed");


    ASSURE_SUCCESS(tx_byte_pool_create(&fx_app_byte_pool, "Fx App Pool",
                                 fx_byte_pool_buffer, FX_APP_MEM_POOL_SIZE), "Fx App Pool create failed");

    ASSURE_SUCCESS(tx_byte_pool_create(&nx_app_byte_pool, "Nx App Pool",
                                 nx_byte_pool_buffer, NX_APP_MEM_POOL_SIZE), "Nx App Pool create failed");

    fx_system_initialize();
    nx_system_initialize();

    void *ptr=0;
    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, NX_APP_PACKET_POOL_SIZE, TX_NO_WAIT), "NetXDuo App Pool allocate failed");
    ASSURE_SUCCESS(nx_packet_pool_create(&g_app_state.packet_pool, "NetXDuo App Pool",
                                DEFAULT_PAYLOAD_SIZE, ptr, NX_APP_PACKET_POOL_SIZE), "NetXDuo App Pool create failed");

    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, Nx_IP_INSTANCE_THREAD_SIZE, TX_NO_WAIT), "IP instance memory allocate failed");
    ASSURE_SUCCESS(nx_ip_create(&g_app_state.ip_instance, "NetX IP",
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
    tx_thread_create(&g_app_state.app_main_thread, "App Main Thread",
                     app_main_thread_entry, 0,
                     (VOID *)ptr, NX_APP_THREAD_STACK_SIZE,
                     NX_APP_THREAD_PRIORITY, NX_APP_THREAD_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, DEFAULT_MEMORY_SIZE, TX_NO_WAIT), "LED Thread stack allocate failed");
    tx_thread_create(&g_app_state.led_thread, "LED Thread",
                     led_thread_entry, 0,
                     (VOID *)ptr, DEFAULT_MEMORY_SIZE,
                     TOGGLE_LED_PRIORITY, TOGGLE_LED_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, 2 * DEFAULT_MEMORY_SIZE, TX_NO_WAIT), "Link Thread stack allocate failed");
    tx_thread_create(&g_app_state.link_thread, "Link Thread",
                     link_thread_entry, 0,
                     (VOID *)ptr, 2 * DEFAULT_MEMORY_SIZE,
                     LINK_PRIORITY, LINK_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, SERVER_POOL_SIZE, TX_NO_WAIT), "HTTP Server Pool allocate failed");
    NX_PACKET_POOL *server_pool = (NX_PACKET_POOL *)ptr;
    ASSURE_SUCCESS(nx_packet_pool_create(server_pool, "HTTP Server Pool",
                                SERVER_PACKET_SIZE, (VOID *)(server_pool + 1),
                                SERVER_POOL_SIZE - sizeof(NX_PACKET_POOL)), "HTTP Server Pool create failed");

    ASSURE_SUCCESS(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, SERVER_STACK, TX_NO_WAIT), "HTTP Server stack allocate failed");
    ASSURE_SUCCESS(nx_web_http_server_create(&g_app_state.http_server, "HTTP Server",
                                    &g_app_state.ip_instance, CONNECTION_PORT,
                                    &g_app_state.sd_media, (VOID *)ptr, SERVER_STACK,
                                    server_pool, NX_NULL,
                                    webserver_request_callback), "HTTP Server create failed");

    ASSURE_SUCCESS(nx_dhcp_create(&g_app_state.dhcp_client, &g_app_state.ip_instance, "DHCP Client"), "DHCP Client create failed");

    printf("Application initialization complete\n");
}
