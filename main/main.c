#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

static QueueHandle_t tcpTxQueue;
static QueueHandle_t tcpRxQueue;

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

    wifi_init_softap();
    tcp_server_init(3333, &tcpTxQueue, &tcpRxQueue);
    can_init(&tcpTxQueue);
}