#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "wifi_network.h"
#include "tcp_server.h"
#include "can.h"
#include "data.h"
#include "frame_parser.h"

#define TAG __func__

static QueueHandle_t tcpTxQueue;
static QueueHandle_t tcpRxQueue;
static QueueHandle_t canRxQueue;
static QueueHandle_t canTxQueue;
static dev_buffer_t devTcpRx;
static TaskHandle_t taskHandle_canRx = NULL;
static TaskHandle_t taskHandle_tcpRx = NULL;

static void canRxHandler(void * pvParameters)
{
    static can_buffer_t rxMsg;
    static dev_buffer_t txTcp;
    static uint8_t formatBuffer[FRAME_TX_OVERHEAD + sizeof(rxMsg)];
    int frameSize = 0;

    txTcp.len = 0;

    while(1) {
        xQueueReceive(canRxQueue, (void *)&rxMsg, portMAX_DELAY);
        /* Format Frame */
        frameSize = format_frame(formatBuffer, sizeof(formatBuffer), (uint8_t *)&rxMsg, sizeof(rxMsg));

        /* Aggregate Rx CAN messages before sending TCP */
        if((txTcp.len + frameSize) < DEVICE_BUFFER_LENGTH) {
            memcpy((void *)&(txTcp.u8Element[txTcp.len]), (void *)formatBuffer, frameSize);
            txTcp.len += frameSize;
        } else {
            /* Send upstream */
            if(pdPASS != xQueueSend(tcpTxQueue, (void*) &txTcp, pdMS_TO_TICKS(100))) {
                ESP_LOGI(TAG, "tcpTx Queue full!");
            }

            txTcp.len = 0;
            memcpy((void *)&(txTcp.u8Element[txTcp.len]), (void *)formatBuffer, frameSize);
            txTcp.len += frameSize;
        }
    }
}


static void tcpRxHandler(void * pvParameters)
{
    static dev_buffer_t rxTcp;
    while(1) {
        xQueueReceive(tcpRxQueue, (void *)&rxTcp, portMAX_DELAY);
    }
}


void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if((ret == ESP_ERR_NVS_NO_FREE_PAGES) || (ret == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    tcpTxQueue = xQueueCreate(100, sizeof(dev_buffer_t));
    tcpRxQueue = xQueueCreate(100, sizeof(dev_buffer_t));
    canTxQueue = xQueueCreate(256, sizeof(can_buffer_t));
    canRxQueue = xQueueCreate(256, sizeof(can_buffer_t));

    wifi_init_softap();
    tcp_server_init(3333, &tcpTxQueue, &tcpRxQueue);
    can_init(&canTxQueue, &canRxQueue);

    xTaskCreate(tcpRxHandler, "tcp_rx_handler", 4096, NULL, 5, &taskHandle_tcpRx);
    xTaskCreate(canRxHandler, "can_rx_handler", 4096, NULL, 5, &taskHandle_tcpRx);

    ESP_LOGI(TAG, "Goodbye");
}