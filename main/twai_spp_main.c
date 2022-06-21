#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "string.h"
#include "bt_bridge.h"
#include "frame_parser.h"

#define RX_TASK_PRIO            8
#define MAIN_BREADCRUMB_DEBUG   0

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

static const char * TAG = "twai_spp";
static bool bTwaiInit = false;
static bool bClientConnected = false;

static void callback_clientConnected(void)
{
    vTaskDelay(500);
    char * name = bt_bridge_get_name();
    /*
     * Format payload
     */
    uint32_t tmpBufLen = strlen(name) + 1;
    uint8_t * tmpBuf = (uint8_t *)malloc(tmpBufLen);
    if(tmpBuf) {
        tmpBuf[0] = CMD_GET_DEVICE_NAME;
        memcpy(&tmpBuf[1], name, strlen(name));
        ESP_ERROR_CHECK(spp_send(tmpBuf, tmpBufLen));
        free(tmpBuf);
        bClientConnected = true;
    } else {
        ESP_LOGE(TAG, "callback_clientConnected: Failed to allocate memory");
    }
}

static void callback_clientDisconnected(void)
{
    bClientConnected = false;
}

static esp_err_t send_to_twai(twai_message_t *pMessage)
{
    return ESP_OK;
}

static void twai_receive_task(void *arg)
{
    twai_message_t rx_msg;
    esp_err_t ret = ESP_OK;
    uint8_t tmpBuf[12]; // 1B CMD + 2B ID + 1B LEN + 8B DATA
    uint8_t idx = 0;

    /* Wait for TWAI to start */
    while(bTwaiInit != true) {
        vTaskDelay(1);
    }

    ESP_LOGI(TAG, "TWAI Rx task running.");

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

        if(bClientConnected == false) {
            continue;
        }

        /*
         * Pass message to BT SPP
         */
        memset(tmpBuf, 0, 12);
        idx = 0;
        tmpBuf[idx++] = CMD_SEND_UPSTREAM;
        tmpBuf[idx++] = (uint8_t)(rx_msg.identifier & 0x00FF);
        tmpBuf[idx++] = (uint8_t)((rx_msg.identifier >> 8) & 0x00FF);
        tmpBuf[idx++] = rx_msg.data_length_code;
        for(int i = 0; i < rx_msg.data_length_code; i++) {
            tmpBuf[idx++] = rx_msg.data[i];
        }
        ret = spp_send(tmpBuf, idx);
        if(ESP_OK != ret) {
            ESP_LOGE(TAG, "Failed spp_send()! err: %d", ret);
        }
    }

    ESP_LOGI(TAG, "TWAI Rx task deleting.");
    vTaskDelete(NULL);
}

void app_main(void)
{
    uint8_t *pRxData = NULL;
    RingbufHandle_t RxBufHdl = NULL;
    size_t nRxData = 0;
    
    /* Initialize NVS */
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    parser_init();

    /* Initialize BT Classic SPP */
    bt_bridge_init();
    bt_bridge_register_srv_open_cb(callback_clientConnected);
    bt_bridge_register_srv_close_cb(callback_clientDisconnected);
    while(1) {
        RxBufHdl = bt_bridge_get_rx_hdl();
        if(RxBufHdl == NULL) {
            vTaskDelay(1);
        } else {
            break;
        }
    }

    /*
     * Initialize TWAI
     */
    /* Create Task */
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    /* Install TWAI driver */
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "TWAI driver installed.");
    /* Start TWAI driver */
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI started.");    
    bTwaiInit = true;

    while(1) {
        /* Check for receive data from BT */
        pRxData = (uint8_t *)xRingbufferReceiveUpTo(RxBufHdl, &nRxData, 5, 64);
        if((pRxData != NULL) && (nRxData > 0)) {
#if (MAIN_BREADCRUMB_DEBUG)
            ESP_LOGI(TAG, "[BT_SPP] Rx Len: %d", nRxData);
#endif
            spp_parser_store(pRxData, nRxData);
            spp_parser_process();
#if (MAIN_BREADCRUMB_DEBUG)
            for(int i = 0; i < nRxData; i++) {
                ESP_LOGI(TAG, "[BT_SPP] Rx: %d", pRxData[i]);
            }
#endif
            vRingbufferReturnItem(RxBufHdl, (void*)pRxData);
        }
    }

    ESP_LOGI(TAG, "Goodbye app_main.");
}