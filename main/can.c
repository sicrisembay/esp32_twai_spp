#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "string.h"
#include "can.h"
#include "tcp_server.h"
#include "data.h"

#define MAIN_BREADCRUMB_DEBUG 1

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
static QueueHandle_t * tcpTxQ;

static void twai_receive_task(void *arg)
{
    twai_message_t rx_msg;
    esp_err_t ret = ESP_OK;
    uint8_t tmpBuf[12]; // 1B CMD + 2B ID + 1B LEN + 8B DATA
    uint8_t idx = 0;
    dev_buffer_t buf;

    /* Wait for TWAI to start */
    while(bTwaiInit != true) {
        vTaskDelay(1);
    }

    ESP_LOGI(TAG, "task running.");

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

        if(tcp_server_socket_connected() != true) {
            continue;
        }

        /*
         * Pass message to socket
         */
        memset(buf.u8Element, 0, sizeof(buf.u8Element));
        idx = 0;
        buf.u8Element[idx++] = CMD_SEND_UPSTREAM;
        buf.u8Element[idx++] = (uint8_t)(rx_msg.identifier & 0x00FF);
        buf.u8Element[idx++] = (uint8_t)((rx_msg.identifier >> 8) & 0x00FF);
        buf.u8Element[idx++] = rx_msg.data_length_code;
        for(int i = 0; i < rx_msg.data_length_code; i++) {
            buf.u8Element[idx++] = rx_msg.data[i];
        }
        
        buf.len = idx;
        xQueueSend(*tcpTxQ, (void *)&buf, portMAX_DELAY);
    }
}


void can_init(QueueHandle_t * tcpTxQueue)
{
    tcpTxQ = tcpTxQueue;

    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, CONFIG_TWAI_RX_TASK_PRIORITY, NULL, tskNO_AFFINITY);
    /* Install TWAI driver */
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "TWAI driver installed.");
    /* Start TWAI driver */
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI started.");    
    bTwaiInit = true;
}