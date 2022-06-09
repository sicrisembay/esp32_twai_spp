#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "string.h"
#include "bt_bridge.h"

#define RX_TASK_PRIO            8

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = {
            .mode = TWAI_MODE_NORMAL,
            .tx_io = CONFIG_TWAI_BRIDGE_TX_GPIO_NUM,
            .rx_io = CONFIG_TWAI_BRIDGE_RX_GPIO_NUM,
            .clkout_io = TWAI_IO_UNUSED,
            .bus_off_io = TWAI_IO_UNUSED,
            .tx_queue_len = 5,
            .rx_queue_len = 5,
            .alerts_enabled = TWAI_ALERT_NONE,
            .clkout_divider = 0,
            .intr_flags = ESP_INTR_FLAG_LEVEL1
};

static const char * TAG = "twai_spp";
static bool bTwaiInit = false;

static void callback_clientConnected(void)
{
    char * name = bt_bridge_get_name();
    bt_bridge_send((uint8_t *)name, strlen(name) + 1);
}

static void callback_writeDone(void)
{
    ESP_LOGI(TAG, "write done");
}

static esp_err_t send_to_bt_spp(twai_message_t *pMessage)
{
    /* Step1: Embed TWAI message into frame */

    /* Step2: Send to BT */
    
    return ESP_OK;
}

static esp_err_t send_to_twai(twai_message_t *pMessage)
{
    return ESP_OK;
}

static void twai_receive_task(void *arg)
{
    twai_message_t rx_msg;
    esp_err_t ret = ESP_OK;

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

        /*
         * Pass message to BT SPP
         */
        ret = send_to_bt_spp(&rx_msg);
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

    /* Initialize BT Classic SPP */
    bt_bridge_init();
    bt_bridge_register_srv_open_cb(callback_clientConnected);
    bt_bridge_register_write_done_cb(callback_writeDone);
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
        if(pRxData != NULL) {
            for(int i = 0; i < nRxData; i++) {
                ESP_LOGI(TAG, "[BT_SPP] Rx: %d", pRxData[i]);
            }
            vRingbufferReturnItem(RxBufHdl, (void*)pRxData);
        }
    }

    ESP_LOGI(TAG, "Goodbye app_main.");
}