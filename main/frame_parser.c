#include "stdint.h"
#include "stddef.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "frame_parser.h"

#define ENABLE_BREADCRUMB_DEBUG (0)

#define FRAME_RX_SZ_BIT     (10)
#define FRAME_RX_SIZE       (1UL << FRAME_RX_SZ_BIT)
#define FRAME_RX_SZ_MASK    (FRAME_RX_SIZE - 1)

#define FRAME_TX_SZ_BIT     (10)
#define FRAME_TX_SIZE       (1UL << FRAME_TX_SZ_BIT)
#define FRAME_TX_SZ_MASK    (FRAME_TX_SIZE - 1)

volatile uint32_t sppRxRdPtr = 0;
volatile uint32_t sppRxWrPtr = 0;
static uint8_t sppRxFrameBuffer[FRAME_RX_SIZE];

static uint32_t sppTxRdPtr = 0;
static uint32_t sppTxWrPtr = 0;
static uint8_t sppTxFrameBuffer[FRAME_TX_SIZE];
static SemaphoreHandle_t xMutexSppTxBuffer;
static uint16_t sppTxSeq = 0;

static const char * TAG = "frame_parser";

/*
 * Process valid frame (Device <-- PC)
 */
static void process_valid_spp_rx_frame(const uint32_t index, uint8_t len)
{
    uint8_t cmd = sppRxFrameBuffer[(index + FRAME_RX_PAYLOAD_CMD_OFFSET) & FRAME_RX_SZ_MASK];
    switch(cmd) {
        case CMD_GET_TWAI_NODE_ID: {
            ESP_LOGI(TAG, "process_valid_spp_rx_frame: cmd: CMD_GET_TWAI_NODE_ID");
            break;
        }
        case CMD_SET_TWAI_BAUDRATE: {
            ESP_LOGI(TAG, "process_valid_spp_rx_frame: cmd: CMD_SET_TWAI_BAUDRATE");
            break;
        }
        case CMD_ENABLE_TWAI: {
            ESP_LOGI(TAG, "process_valid_spp_rx_frame: cmd: CMD_ENABLE_TWAI");
            break;
        }
        case CMD_DISABLE_TWAI: {
            ESP_LOGI(TAG, "process_valid_spp_rx_frame: cmd: CMD_DISABLE_TWAI");
            break;
        }
        case CMD_SEND_DOWNSTREAM: {
#if (ENABLE_BREADCRUMB_DEBUG)
            ESP_LOGI(TAG, "process_valid_spp_rx_frame: cmd: CMD_SEND_DOWNSTREAM");
#endif
            twai_message_t downstream_msg;
            esp_err_t retval = ESP_OK;
            downstream_msg.flags = 0;
            downstream_msg.identifier = sppRxFrameBuffer[(index + FRAME_RX_PAYLOAD_PARAM_OFFSET) & FRAME_RX_SZ_MASK];
            downstream_msg.identifier |= (((uint16_t)(sppRxFrameBuffer[(index + FRAME_RX_PAYLOAD_PARAM_OFFSET + 1) & FRAME_RX_SZ_MASK])) << 8);
            downstream_msg.data_length_code = sppRxFrameBuffer[(index + FRAME_RX_PAYLOAD_PARAM_OFFSET + 2) & FRAME_RX_SZ_MASK];
            for(int i = 0; i < downstream_msg.data_length_code; i++) {
                downstream_msg.data[i] = sppRxFrameBuffer[(index + FRAME_RX_PAYLOAD_PARAM_OFFSET + 3 + i) & FRAME_RX_SZ_MASK];
            }
            retval = twai_transmit(&downstream_msg, pdMS_TO_TICKS(2000));
            if(ESP_OK != retval) {
                ESP_LOGI(TAG, "Failed twai_transmit 0x%X", retval);
            }
            break;
        }
        case CMD_SEND_UPSTREAM: {
            ESP_LOGI(TAG, "process_valid_spp_rx_frame: cmd: CMD_SEND_UPSTREAM");
            break;
        }
        default: {
            ESP_LOGI(TAG, "process_valid_spp_rx_frame: unknown cmd: 0x%02X", cmd);
            break;
        }
    }
}


void parser_init()
{
    sppRxRdPtr = 0;
    sppRxWrPtr = 0;
    memset(sppRxFrameBuffer, 0, FRAME_RX_SIZE);

    /*
     * Init SPP Tx Buffers
     */
    sppTxRdPtr = 0;
    sppTxWrPtr = 0;
    memset(sppTxFrameBuffer, 0, FRAME_TX_SIZE);
    sppTxSeq = 0;
    xMutexSppTxBuffer = xSemaphoreCreateRecursiveMutex();
    if (NULL == xMutexSppTxBuffer) {
        /* Failed to create mutex */
        ESP_ERROR_CHECK(ESP_FAIL);
    }
}

esp_err_t spp_tx_get_block(uint8_t * pBlock, uint32_t * pSize)
{
    if ((pBlock == NULL) || (pSize == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (*pSize == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint32_t used = (sppTxWrPtr >= sppTxRdPtr) ? (sppTxWrPtr - sppTxRdPtr) :
                    (FRAME_TX_SIZE - (sppTxRdPtr - sppTxWrPtr));

    uint32_t bytes_to_copy;
    if(*pSize >= used) {
        bytes_to_copy = used;
    } else {
        bytes_to_copy = *pSize;
    }

    for (uint32_t i = 0; i < bytes_to_copy; i++) {
        pBlock[i] = sppTxFrameBuffer[sppTxRdPtr];
        sppTxRdPtr = (sppTxRdPtr + 1) & FRAME_TX_SZ_MASK;
    }

    *pSize = bytes_to_copy;

    return ESP_OK;
}


esp_err_t spp_send(uint8_t * pBuf, uint32_t len)
{
    uint16_t frameLen = 0;
    uint32_t timestamp = 0;
    uint8_t checksum = 0;

    if ((NULL == pBuf) || (len == 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    frameLen = len + FRAME_TX_OVERHEAD;
    if (frameLen > 255) {
        /* Payload length too big */
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Add check that the whole packet can fit into Tx Buffer
     */
    uint32_t used = (sppTxWrPtr >= sppTxRdPtr) ? (sppTxWrPtr - sppTxRdPtr) :
                        (FRAME_TX_SIZE - (sppTxRdPtr - sppTxWrPtr));
    uint32_t freeInBuf = (FRAME_TX_SIZE - 1) - used;
#if (ENABLE_BREADCRUMB_DEBUG)
    ESP_LOGI(TAG, "Spp Tx Buff Free: %d", freeInBuf);
#endif
    if (frameLen > freeInBuf) {
        /* Unable to fit into buffer */
        return ESP_ERR_NO_MEM;
    }

    if(pdTRUE != xSemaphoreTakeRecursive(xMutexSppTxBuffer, 10)) {
        /* Unable to get mutex and timed out */
        return ESP_ERR_TIMEOUT;
    }

    /* SOF */
    sppTxFrameBuffer[sppTxWrPtr] = TAG_SOF;
    checksum = TAG_SOF;
    sppTxWrPtr = (sppTxWrPtr + 1) & FRAME_TX_SZ_MASK;
    /* LEN */
    sppTxFrameBuffer[sppTxWrPtr] = frameLen;
    checksum += sppTxFrameBuffer[sppTxWrPtr];
    sppTxWrPtr = (sppTxWrPtr + 1) & FRAME_TX_SZ_MASK;
    /* Timestamp */
    timestamp = xTaskGetTickCount();
    sppTxFrameBuffer[sppTxWrPtr] = timestamp & 0x000000FF;
    checksum += sppTxFrameBuffer[sppTxWrPtr];
    sppTxWrPtr = (sppTxWrPtr + 1) & FRAME_TX_SZ_MASK;
    sppTxFrameBuffer[sppTxWrPtr] = (timestamp >> 8) & 0x000000FF;
    checksum += sppTxFrameBuffer[sppTxWrPtr];
    sppTxWrPtr = (sppTxWrPtr + 1) & FRAME_TX_SZ_MASK;
    sppTxFrameBuffer[sppTxWrPtr] = (timestamp >> 16) & 0x000000FF;
    checksum += sppTxFrameBuffer[sppTxWrPtr];
    sppTxWrPtr = (sppTxWrPtr + 1) & FRAME_TX_SZ_MASK;
    sppTxFrameBuffer[sppTxWrPtr] = (timestamp >> 24) & 0x000000FF;
    checksum += sppTxFrameBuffer[sppTxWrPtr];
    sppTxWrPtr = (sppTxWrPtr + 1) & FRAME_TX_SZ_MASK;
    /* Packet SEQ */
    sppTxFrameBuffer[sppTxWrPtr] = sppTxSeq & 0x00FF;
    checksum += sppTxFrameBuffer[sppTxWrPtr];
    sppTxWrPtr = (sppTxWrPtr + 1) & FRAME_TX_SZ_MASK;
    sppTxFrameBuffer[sppTxWrPtr] = (sppTxSeq >> 8) & 0x00FF;
    checksum += sppTxFrameBuffer[sppTxWrPtr];
    sppTxWrPtr = (sppTxWrPtr + 1) & FRAME_TX_SZ_MASK;
    sppTxSeq++;
    /* Payload */
    for (uint32_t i = 0; i < len; i++) {
        sppTxFrameBuffer[sppTxWrPtr] = pBuf[i];
        checksum += sppTxFrameBuffer[sppTxWrPtr];
        sppTxWrPtr = (sppTxWrPtr + 1) & FRAME_TX_SZ_MASK;
    }
    /* Checksum */
    sppTxFrameBuffer[sppTxWrPtr] = -checksum;
    sppTxWrPtr = (sppTxWrPtr + 1) & FRAME_TX_SZ_MASK;

    xSemaphoreGiveRecursive(xMutexSppTxBuffer);
#if (ENABLE_BREADCRUMB_DEBUG)
    ESP_LOGI(TAG, "spp_send (Wr: %d, Rd: %d)", sppTxWrPtr, sppTxRdPtr);
#endif

    return ESP_OK;
}


void spp_parser_store(uint8_t * pBuf, uint32_t len)
{
    if((pBuf != NULL) && (len > 0)) {
        for(uint32_t i = 0; i < len; i++) {
            sppRxFrameBuffer[sppRxWrPtr] = pBuf[i];
            sppRxWrPtr = (sppRxWrPtr + 1) & FRAME_RX_SZ_MASK;
        }
#if (ENABLE_BREADCRUMB_DEBUG)
        ESP_LOGI(TAG, "Spp Rx Buff RdIdx: %d WrIdx: %d", sppRxRdPtr, sppRxWrPtr);
#endif
    }
}

/*
 * Parse Frame (Device <-- PC)
 */
void spp_parser_process()
{
    uint32_t availableBytes = 0;
    uint8_t length = 0;
    uint32_t idx = 0;
    uint8_t sum = 0;

    while(sppRxWrPtr != sppRxRdPtr) {
        /* Check Start of Frame */
        if (TAG_SOF != sppRxFrameBuffer[sppRxRdPtr]) {
            /* Not SOF, skip character */
            sppRxRdPtr = (sppRxRdPtr + 1) & FRAME_RX_SZ_MASK;
            continue;
        }

        /* Get available bytes in the buffer */
        if (sppRxWrPtr >= sppRxRdPtr) {
            availableBytes = sppRxWrPtr - sppRxRdPtr;
        } else {
            availableBytes = (sppRxWrPtr + FRAME_RX_SIZE) - sppRxRdPtr;
        }
        if (availableBytes < FRAME_RX_OVERHEAD) {
            /*
             * Less than minimum bytes needed to proceed
             */
            break;
        }
        /*
         * Check if packet size is valid
         * Note: Length is a lenght of the packet and not the payload.
         *       (includes SOF)
         */
        length = sppRxFrameBuffer[(sppRxRdPtr + FRAME_RX_LEN_OFFSET) & FRAME_RX_SZ_MASK];
        if ((length < FRAME_RX_OVERHEAD) || (length > 127)) {
            /*
             * Packet length minimum is FRAME_RX_OVERHEAD
             * 1 SOF + 1 LEN + 1 CHKSM
             * 
             * Packet length maximum is 127
             */
            sppRxRdPtr = (sppRxRdPtr + 1) & FRAME_RX_SZ_MASK;
            continue;
        }
        /*
         * Check if entire packet is in the buffer
         */
        if (availableBytes < length) {
            /* not yet received entire packet */
            break;
        }
        /*
         * Received entire packet
         * Check packet integrity
         */
        for (idx = 0, sum = 0; idx < length; idx++) {
            sum += sppRxFrameBuffer[(sppRxRdPtr + idx) & FRAME_RX_SZ_MASK];
        }
        /*
         * Skip this packet if the checksum is not correct.
         * It is probably not really the start of packet.
         */
        if (sum != 0) {
            /* Skip this character */
            sppRxRdPtr = (sppRxRdPtr + 1) & FRAME_RX_SZ_MASK;
            /* Keep scanning for SOF */
            continue;
        }
        /*
         * It is a valid frame.
         * Process frame
         */
        process_valid_spp_rx_frame(sppRxRdPtr, length);

        /*
         * Done with processing this packet
         */
        sppRxRdPtr = (sppRxRdPtr + length) & FRAME_RX_SZ_MASK;
    }
}