#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "bt_bridge.h"
#include "freertos/ringbuf.h"
#include "frame_parser.h"

#include "time.h"
#include "sys/time.h"

static const char *TAG = "bt_bridge";
#define SPP_SERVER_NAME "SPP_SERVER"

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static uint32_t bt_initiator_hdl = 0;
static RingbufHandle_t bridgeRxBuf_handle = NULL;
static bool bCongest = false;
static bool bBtWriteDone = true;
static bool bInit = false;
static bt_bridge_srv_open_cb_t srv_open_cb = NULL;
static bt_bridge_srv_close_cb_t srv_close_cb = NULL;
static bt_bridge_write_done_cb_t write_done_cb = NULL;
static char btDevName[33];

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT: {
        ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
        snprintf(btDevName, 32, "ESP_BT_CAN_%02X:%02X:%02X:%02X:%02X:%02X", ESP_BD_ADDR_HEX(esp_bt_dev_get_address()));
        esp_bt_dev_set_device_name(btDevName);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);
        break;
    }
    case ESP_SPP_UNINIT_EVT: {
        ESP_LOGI(TAG, "ESP_SPP_UNINIT_EVT");
        break;
    }
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CLOSE_EVT");
        ESP_LOGI(TAG, "Client Handle=%d", param->close.handle);
        bt_initiator_hdl = 0;
        if(srv_close_cb != NULL) {
            srv_close_cb();
        }
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT: {
#if 0
        ESP_LOGI(TAG, "ESP_SPP_DATA_IND_EVT (Client Hdl=%d, data=%d)", param->data_ind.handle, *(param->data_ind.data));
#endif
        if(xRingbufferGetCurFreeSize(bridgeRxBuf_handle) >= param->data_ind.len) {
            xRingbufferSend(bridgeRxBuf_handle, param->data_ind.data, param->data_ind.len, (TickType_t)0);
        } else {
            ESP_LOGW(TAG, "BT Rx buffer full");
        }
        break;
    }
    case ESP_SPP_CONG_EVT:
        ESP_LOGW(TAG, "ESP_SPP_CONG_EVT");
        bCongest = param->cong.cong;
        break;
    case ESP_SPP_WRITE_EVT:
#if 0
        ESP_LOGI(TAG, "ESP_SPP_WRITE_EVT");
#endif
        bBtWriteDone = true;
        if(write_done_cb != NULL) {
            write_done_cb(bCongest);
        }
        break;
    case ESP_SPP_SRV_OPEN_EVT: {
        ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT");
        ESP_LOGI(TAG, "client handle=%d new handle=%d", param->srv_open.handle, param->srv_open.new_listen_handle);
        bt_initiator_hdl = param->srv_open.handle;
        if(srv_open_cb != NULL) {
            srv_open_cb();
        }
        break;
    }
    default:
        ESP_LOGI(TAG, "spp event: %d", event);
        break;
    }
}

static void bt_bridge_task(void * arg)
{
    uint32_t blockLen;
    uint8_t txBuf[256];

    while(1) {
        blockLen = 256;
        if(!bt_bridge_rdy() || bt_bride_is_congested()) {
            vTaskDelay(1);
        } else {
            spp_tx_get_block(txBuf, &blockLen);
            if(blockLen != 0) {
                bt_bridge_send(txBuf, blockLen);
            } else {
                vTaskDelay(1);
            }
        }
    }
    vTaskDelete(NULL);
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }
    default: {
        ESP_LOGI(TAG, "gap event: %d", event);
        break;
    }
    }
    return;
}


void bt_bridge_init(void)
{
    esp_bt_cod_t cod;
    esp_err_t ret;

    if(bInit != true) {
        /* Task */
        xTaskCreatePinnedToCore(bt_bridge_task, "BT_bridge", 4096, NULL, 7, NULL, tskNO_AFFINITY);

        /* BT receive buffer */
        bridgeRxBuf_handle = xRingbufferCreate(CONFIG_BT_BRIDGE_RX_BUFSZ, RINGBUF_TYPE_BYTEBUF);
        if(bridgeRxBuf_handle == NULL) {
            ESP_LOGE(TAG, "%s bridge rx buff creation failed: %s\n", __func__, "ESP_ERR_NO_MEM");
            return;
        }

        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
            ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }

        if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
            ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }

        if ((ret = esp_bluedroid_init()) != ESP_OK) {
            ESP_LOGE(TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }

        if ((ret = esp_bluedroid_enable()) != ESP_OK) {
            ESP_LOGE(TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }

        if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
            ESP_LOGE(TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }

        /* refer to
         * https://www.bluetooth.com/specifications/assigned-numbers/baseband
         */
        cod.major = 0b00001;
        cod.minor = 0b000101;
        cod.service = 0b00000010000;
        if ((ret = esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD)) != ESP_OK) {
            ESP_LOGE(TAG, "%s Failed setting COD: %s\n", __func__, esp_err_to_name(ret));
            return;
        }

        if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
            ESP_LOGE(TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }

        if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
            ESP_LOGE(TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }

        /*
         * Set default parameters for Legacy Pairing
         * Use variable pin, input pin code when pairing
         */
        esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
        esp_bt_pin_code_t pin_code;
        esp_bt_gap_set_pin(pin_type, 0, pin_code);

        bInit = true;
    }
}

void bt_bridge_register_srv_open_cb(bt_bridge_srv_open_cb_t callback)
{
    srv_open_cb = callback;
}

void bt_bridge_register_srv_close_cb(bt_bridge_srv_close_cb_t callback)
{
    srv_close_cb = callback;
}

void bt_bridge_register_write_done_cb(bt_bridge_write_done_cb_t callback)
{
    write_done_cb = callback;
}

RingbufHandle_t bt_bridge_get_rx_hdl(void)
{
    RingbufHandle_t retval = NULL;
    if(bInit == true) {
        retval = bridgeRxBuf_handle;
    } else {
        ESP_LOGW(TAG, "%s called before bt_bridge_init", __func__);
    }
    return(retval);
}

char * bt_bridge_get_name(void)
{
    return btDevName;
}

bool bt_bridge_rdy(void)
{
    return (bBtWriteDone && (bt_initiator_hdl != 0) && bInit);
}

bool bt_bride_is_congested(void)
{
    return bCongest;
}

esp_err_t bt_bridge_send(uint8_t * pData, uint16_t size)
{
    esp_err_t retval = ESP_OK;
    if(pData == NULL) {
        retval = ESP_ERR_INVALID_ARG;
    } else {
        if(bt_bridge_rdy()) {
            if(bCongest) {
                ESP_LOGW(TAG, "bt_bridge_send: Congested!");
                retval = ESP_ERR_INVALID_STATE;
            } else {
                retval = esp_spp_write(bt_initiator_hdl, size, pData);
                if(retval == ESP_OK) {
                    bBtWriteDone = false;
                } else {
                    ESP_LOGW(TAG, "esp_spp_write failed: %s", esp_err_to_name(retval));
                }
            }
        } else {
            ESP_LOGW(TAG, "bt_bridge_send: bt_bridge_rdy() false!");
            retval = ESP_ERR_INVALID_STATE;
        }
    }
    return (retval);
}