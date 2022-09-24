#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "string.h"
#include "can.h"
#include "tcp_server.h"
#include "data.h"
#include "frame_parser.h"

#define MAIN_BREADCRUMB_DEBUG 0

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = {
            .mode = TWAI_MODE_NORMAL,
            .tx_io = CONFIG_TWAI_BRIDGE_TX_GPIO_NUM,
            .rx_io = CONFIG_TWAI_BRIDGE_RX_GPIO_NUM,
            .clkout_io = TWAI_IO_UNUSED,
            .bus_off_io = TWAI_IO_UNUSED,
            .tx_queue_len = 512,
            .rx_queue_len = 512,
            .alerts_enabled = TWAI_ALERT_NONE,
            .clkout_divider = 0,
            .intr_flags = ESP_INTR_FLAG_LEVEL1
};

#define TAG __func__

static bool bTwaiInit = false;
static QueueHandle_t * canTxQ = NULL;
static QueueHandle_t * canRxQ = NULL;
static SemaphoreHandle_t can_mutex;
static TaskHandle_t taskHandle_tx = NULL;
static TaskHandle_t taskHandle_rx = NULL;

static void twai_transmit_task(void *arg)
{
    static twai_message_t canTxMsg;
    static can_buffer_t canTx;
    esp_err_t retval = ESP_OK;


    /* Wait for TWAI to start */
    while(bTwaiInit != true) {
        vTaskDelay(1);
    }

    ESP_LOGI(TAG, "running");

    while(1) {
        xQueueReceive(*canTxQ, (void *)&canTx, portMAX_DELAY);

        if(canTx.dlc > CAN_STANDARD_BUFFER_LENGTH) {
            /* Standard CAN has max 8 bytes */
            continue;
        }

        memset(&canTxMsg, 0, sizeof(canTxMsg));

        /* Take Mutex */
        if(xSemaphoreTake(can_mutex, 100) == pdTRUE) {
            canTxMsg.identifier = canTx.identifier;
            canTxMsg.data_length_code = canTx.dlc;
            for(int i = 0; i < canTx.dlc; i++) {
                canTxMsg.data[i] = canTx.data[i];
            }
            retval = twai_transmit(&canTxMsg, pdMS_TO_TICKS(500));
            if(ESP_OK != retval) {
                ESP_LOGI(TAG, "Failed to transmit %d", retval);
            }
            /* Release Mutex */
            xSemaphoreGive(can_mutex);
        }
    }
}

static void twai_receive_task(void *arg)
{
    static twai_message_t rx_msg;
    static can_buffer_t canRx;
    uint8_t idx = 0;

    /* Wait for TWAI to start */
    while(bTwaiInit != true) {
        vTaskDelay(1);
    }

    ESP_LOGI(TAG, "running.");

    while(1) {
        /*
         * Wait to Receive TWAI message
         */
        twai_receive(&rx_msg, portMAX_DELAY);
#if (MAIN_BREADCRUMB_DEBUG)
        ESP_LOGI(TAG, "TWAI id: 0x%03X, len: %d", rx_msg.identifier, (int)rx_msg.data_length_code);
        for(int i = 0; i < rx_msg.data_length_code; i++) {
            ESP_LOGI(TAG, "data%d: 0x%02X", i, rx_msg.data[i]);
        }
#endif
        /*
         * Only process standard frame
         */
        if(rx_msg.extd == 1) {
            continue;
        }

        /*
         * Only process CAN message when someone is connected
         * to the tcp server
         */
        if(tcp_server_socket_connected() != true) {
            continue;
        }

        memset(&canRx, 0, sizeof(canRx));

        /* Take Mutex */
        if(xSemaphoreTake(can_mutex, 100) == pdTRUE) {
            canRx.identifier = rx_msg.identifier;
            canRx.dlc = rx_msg.data_length_code;
            for(idx = 0; idx < canRx.dlc; idx++) {
                canRx.data[idx] = rx_msg.data[idx];
            }
            if(pdPASS != xQueueSend(*canRxQ, (void*) &canRx, pdMS_TO_TICKS(10))) {
                ESP_LOGI(TAG, "canRxQ full!");
            }
            /* Release Mutex */
            xSemaphoreGive(can_mutex);
        }
    }
}


void can_init(QueueHandle_t * canTxQueue, QueueHandle_t * canRxQueue)
{
    if((canTxQueue == NULL) || (canRxQueue == NULL)) {
        ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
    }

    canTxQ = canTxQueue;
    canRxQ = canRxQueue;
    can_mutex = xSemaphoreCreateMutex();

    xTaskCreate(twai_receive_task, "can_rx", 4096, NULL, CONFIG_TWAI_RX_TASK_PRIORITY, &taskHandle_rx);
    xTaskCreate(twai_transmit_task, "can_tx", 4096, NULL, CONFIG_TWAI_RX_TASK_PRIORITY, &taskHandle_tx);
    /* Install TWAI driver */
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "TWAI driver installed.");
    /* Start TWAI driver */
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI started.");    
    bTwaiInit = true;
}