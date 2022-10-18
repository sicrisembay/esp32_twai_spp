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

#define MAX_SLOT_CAN_PERIODIC      (4)
#define MINIMUM_PERIOD_MS          (20)

typedef struct {
    bool bEnable;
    uint16_t period_ms;
    uint16_t counter;
    can_buffer_t msg;
} can_buffer_periodic_slot_t;

static can_buffer_periodic_slot_t can_buffer_periodic[MAX_SLOT_CAN_PERIODIC];

static QueueHandle_t tcpTxQueue;
static QueueHandle_t tcpRxQueue;
static QueueHandle_t canRxQueue;
static QueueHandle_t canTxQueue;
static dev_buffer_t devTcpRx;
static TaskHandle_t taskHandle_canRx = NULL;
static TaskHandle_t taskHandle_tcpRx = NULL;
static TaskHandle_t taskHandle_canPeriodicTx = NULL;
static SemaphoreHandle_t canPeriodicMutex = NULL;

static void canPeriodicTx(void * pvParameters)
{
    uint8_t slotIdx = 0;
    TickType_t xLastWakeTime;
    memset(can_buffer_periodic, 0, sizeof(can_buffer_periodic));

    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        xTaskDelayUntil(&xLastWakeTime, 1);
        if(xSemaphoreTake(canPeriodicMutex, 20) == pdTRUE) {
            for(slotIdx = 0; slotIdx < MAX_SLOT_CAN_PERIODIC; slotIdx++) {
                if(can_buffer_periodic[slotIdx].bEnable) {
                    can_buffer_periodic[slotIdx].counter++;
                    if(can_buffer_periodic[slotIdx].counter >= can_buffer_periodic[slotIdx].period_ms) {
                        /* Send to TWAI */
                        if(pdPASS != xQueueSend(
                                        canTxQueue,
                                        (void*) &(can_buffer_periodic[slotIdx].msg),
                                         2)) {
                            ESP_LOGI(TAG, "canPeriodicTx: canTx Queue full!");
                        }
                        can_buffer_periodic[slotIdx].counter = 0;
                    }
                }
            }
            xSemaphoreGive(canPeriodicMutex);
        } else {
            ESP_LOGW(TAG, "canPeriodicTx failed to get mutex!");
        }
    }
}


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


static void valid_frame_cb(uint8_t * pFrame, uint32_t len)
{
    switch(pFrame[0]) {
        case CMD_SEND_DOWNSTREAM: {
            ESP_LOGI(TAG, "Received: CMD_SEND_DOWNSTREAM frame");
            can_buffer_t cantx;
            memset((void*)&cantx, 0, sizeof(cantx));
            cantx.identifier = pFrame[1] + 
                        ((uint32_t)pFrame[2] << 8) +
                        ((uint32_t)pFrame[3] << 16) +
                        ((uint32_t)pFrame[4] << 24);
            cantx.dlc = pFrame[5];
            if(cantx.dlc > CAN_STANDARD_BUFFER_LENGTH) {
                return;
            }
            if(cantx.dlc > 0) {
                for(int i = 0; i < cantx.dlc; i++) {
                    cantx.data[i] = pFrame[6 + i];
                }
            }
            /* Send downstream */
            if(pdPASS != xQueueSend(canTxQueue, (void*) &cantx, pdMS_TO_TICKS(100))) {
                ESP_LOGI(TAG, "canTx Queue full!");
            }
            break;
        }
        case CMD_SEND_DOWNSTREAM_PERIODIC: {
            ESP_LOGI(TAG, "Received: CMD_SEND_DOWNSTREAM_PERIODIC frame");
            uint8_t slot = pFrame[1];
            if(slot < MAX_SLOT_CAN_PERIODIC) {
                if(xSemaphoreTake(canPeriodicMutex, 100) == pdTRUE) {
                    uint16_t period = (uint16_t)pFrame[3] +
                                   (((uint16_t)pFrame[4]) << 8);
                    uint32_t can_id = pFrame[5] +
                                    ((uint32_t)pFrame[6] << 8) +
                                    ((uint32_t)pFrame[7] << 16) +
                                    ((uint32_t)pFrame[8] << 24);
                    can_buffer_periodic[slot].bEnable = pFrame[2];
                    can_buffer_periodic[slot].counter = 0;
                    can_buffer_periodic[slot].period_ms = period;
                    can_buffer_periodic[slot].msg.identifier = can_id;
                    can_buffer_periodic[slot].msg.dlc = pFrame[9];
                    if(can_buffer_periodic[slot].msg.dlc > 0) {
                        for(int i = 0; i < can_buffer_periodic[slot].msg.dlc; i++) {
                            can_buffer_periodic[slot].msg.data[i] = pFrame[10 + i];
                        }
                    }
                    xSemaphoreGive(canPeriodicMutex);
                } else {
                    ESP_LOGW(TAG, "valid_frame_cb: failed to take mutex!");
                }
            }
            break;
        }
        default: {
            break;
        }
    }
}


static void tcpRxHandler(void * pvParameters)
{
    static dev_buffer_t rxTcp;
    parser_init(valid_frame_cb);

    while(1) {
        xQueueReceive(tcpRxQueue, (void *)&rxTcp, portMAX_DELAY);
        parse_frame(&rxTcp);
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

    canPeriodicMutex = xSemaphoreCreateMutex();
    xTaskCreate(canPeriodicTx, "can_tx_periodic", 4096, NULL, 5, &taskHandle_canPeriodicTx);

#if (CONFIG_LED_GPIO_PIN >= 0)
    gpio_config_t led_conf;
    led_conf.intr_type = GPIO_INTR_DISABLE;
    led_conf.mode = GPIO_MODE_OUTPUT;
    led_conf.pin_bit_mask = (1ULL << CONFIG_LED_GPIO_PIN);
    led_conf.pull_down_en = 0;
    led_conf.pull_up_en = 1;
    gpio_config(&led_conf);

    while(1) {
        gpio_set_level(CONFIG_LED_GPIO_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(CONFIG_LED_GPIO_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(4950));
    }
#endif /* (CONFIG_LED_GPIO_PIN >= 0) */

    ESP_LOGI(TAG, "Goodbye");
}