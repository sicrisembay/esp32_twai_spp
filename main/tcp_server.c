#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "tcp_server.h"
#include "data.h"

#define TCP_RX_BREADCRUMB_DBG       (1)
#define TCP_TX_BREADCRUMB_DBG       (1)
#define KEEPALIVE_IDLE              5
#define KEEPALIVE_INTERVAL          5
#define KEEPALIVE_COUNT             3

#define PORT_CLOSED_BIT         BIT0
#define PORT_OPEN_BIT           BIT1

#define TAG __func__

static uint32_t server_port = 0;
static int sock = -1;
int listen_sock;
static EventGroupHandle_t xSocketEventGroup;
static QueueHandle_t *xTX_Queue, *xRX_Queue;
static SemaphoreHandle_t xTCP_Socket_Mutex;
TaskHandle_t taskHandle_server = NULL;
TaskHandle_t taskHandle_tx = NULL;
TaskHandle_t taskHandle_rx = NULL;
static bool socketConnected = false;

static void tcp_server_rx_task(void *pvParameters)
{
    static dev_buffer_t rx_buffer;

wait_skt_rx:
    xEventGroupWaitBits(
                        xSocketEventGroup,   /* The event group being tested. */
                        PORT_OPEN_BIT, /* The bits within the event group to wait for. */
                        pdFALSE,        /* BIT_0 & BIT_4 should be cleared before returning. */
                        pdFALSE,       /* Don't wait for both bits, either bit will do. */
                        portMAX_DELAY );/* Wait a maximum of 100ms for either bit to be set. */
    
    while(1) {
        rx_buffer.len = recv(sock, rx_buffer.u8Element, sizeof(rx_buffer.u8Element) - 1, 0);
        if( xSemaphoreTake( xTCP_Socket_Mutex, portMAX_DELAY ) == pdTRUE ) {
            //check if sock still connected?
            if (rx_buffer.len < 0) {
                xEventGroupSetBits( xSocketEventGroup, PORT_CLOSED_BIT );
                xEventGroupClearBits( xSocketEventGroup, PORT_OPEN_BIT );
                ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
                xSemaphoreGive( xTCP_Socket_Mutex );
                goto wait_skt_rx;
            } else if (rx_buffer.len == 0) {
                xEventGroupSetBits( xSocketEventGroup, PORT_CLOSED_BIT );
                xEventGroupClearBits( xSocketEventGroup, PORT_OPEN_BIT );
                ESP_LOGW(TAG, "Connection closed");
                xSemaphoreGive( xTCP_Socket_Mutex );
                goto wait_skt_rx;
            } else {
                rx_buffer.u8Element[rx_buffer.len] = 0; // Null-terminate whatever is received and treat it like a string
                xQueueSend( *xRX_Queue, ( void * ) &rx_buffer, portMAX_DELAY );
            }
            xSemaphoreGive( xTCP_Socket_Mutex );
        }
    }
}


static void tcp_server_tx_task(void *pvParameters)
{
    static dev_buffer_t tx_buffer;
wait_skt_tx:
    xEventGroupWaitBits(
                      xSocketEventGroup,   /* The event group being tested. */
                      PORT_OPEN_BIT, /* The bits within the event group to wait for. */
                      pdFALSE,        /* BIT_0 & BIT_4 should be cleared before returning. */
                      pdFALSE,       /* Don't wait for both bits, either bit will do. */
                      portMAX_DELAY );/* Wait a maximum of 100ms for either bit to be set. */
    ESP_LOGI(TAG, "Socket connected...");
    while(1) {
        xQueuePeek(*xTX_Queue, ( void * ) &tx_buffer, portMAX_DELAY);

        while(xQueuePeek(*xTX_Queue, ( void * ) &tx_buffer, 0) == pdTRUE) {
            if( xSemaphoreTake( xTCP_Socket_Mutex, portMAX_DELAY ) == pdTRUE ) {
                int to_write = tx_buffer.len;
                xQueueReceive(*xTX_Queue, ( void * ) &tx_buffer, 0);
#if TCP_TX_BREADCRUMB_DBG
                ESP_LOGI(TAG, "Sending %d bytes", tx_buffer.len);
#endif
                while (to_write > 0) {
                    int written = send(sock, tx_buffer.u8Element + (tx_buffer.len - to_write), to_write, 0);
                    if (written < 0) {
                        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                        xEventGroupSetBits( xSocketEventGroup, PORT_CLOSED_BIT );
                        xEventGroupClearBits( xSocketEventGroup, PORT_OPEN_BIT );
                        xSemaphoreGive( xTCP_Socket_Mutex );
                        goto wait_skt_tx;
                    }
                    to_write -= written;
                }
            }
            xSemaphoreGive( xTCP_Socket_Mutex );
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;
    struct sockaddr_storage source_addr;
    socklen_t addr_len = sizeof(source_addr);

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(server_port);
    ip_protocol = IPPROTO_IP;

    listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);

    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", server_port);

    err = listen(listen_sock, 1);
    if (err != 0)
    {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while(1) {
        ESP_LOGI(TAG, "Socket listening");
accept_socket:
        sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            goto accept_socket;
        }
        xEventGroupClearBits(xSocketEventGroup, PORT_CLOSED_BIT);
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);
        xEventGroupSetBits( xSocketEventGroup, PORT_OPEN_BIT );
        socketConnected = true;
        xEventGroupWaitBits(
                    xSocketEventGroup,   /* The event group being tested. */
                    PORT_CLOSED_BIT, /* The bits within the event group to wait for. */
                    pdFALSE,        /* BIT_0 & BIT_4 should be cleared before returning. */
                    pdFALSE,       /* Don't wait for both bits, either bit will do. */
                    portMAX_DELAY );/* Wait a maximum of 100ms for either bit to be set. */
        xEventGroupClearBits( xSocketEventGroup, PORT_OPEN_BIT );
        socketConnected = false;
        ESP_LOGI(TAG, "Socket disconnected...");
        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}


void tcp_server_init(uint32_t port, QueueHandle_t *txQueue, QueueHandle_t *rxQueue)
{
    server_port = port;
    xTX_Queue = txQueue;
    xRX_Queue = rxQueue;
    xTCP_Socket_Mutex = xSemaphoreCreateMutex();
    xSocketEventGroup = xEventGroupCreate();
    xEventGroupSetBits( xSocketEventGroup, PORT_CLOSED_BIT );
    xEventGroupClearBits( xSocketEventGroup, PORT_OPEN_BIT );
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, &taskHandle_server);
    xTaskCreate(tcp_server_rx_task, "tcp_rx_server", 4096, NULL, 5, &taskHandle_rx);
    xTaskCreate(tcp_server_tx_task, "tcp_tx_server", 4096, NULL, 5, &taskHandle_tx);
}


bool tcp_server_socket_connected(void)
{
    return socketConnected;
}