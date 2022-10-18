#include "stdint.h"
#include "stddef.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "data.h"
#include "frame_parser.h"

#define ENABLE_BREADCRUMB_DEBUG (0)

#define FRAME_RX_SZ_BIT     (10)
#define FRAME_RX_SIZE       (1UL << FRAME_RX_SZ_BIT)
#define FRAME_RX_SZ_MASK    (FRAME_RX_SIZE - 1)

#define FRAME_TX_SZ_BIT     (10)
#define FRAME_TX_SIZE       (1UL << FRAME_TX_SZ_BIT)
#define FRAME_TX_SZ_MASK    (FRAME_TX_SIZE - 1)

static uint16_t sppTxSeq = 0;

static pfn_valid_frame_cb validFrameCb = NULL;

#define TAG __func__

void parser_init(pfn_valid_frame_cb cb)
{
    validFrameCb = cb;
}


int format_frame(uint8_t * dstBuf, size_t dstBufSz, uint8_t * payload, uint32_t payloadLen)
{
    uint16_t frameLen = 0;
    uint32_t timestamp = 0;
    uint8_t checksum = 0;
    int idx = 0;

    if ((NULL == dstBuf) || (NULL == payload) || (payloadLen == 0)) {
        return -1;
    }

    frameLen = payloadLen + FRAME_TX_OVERHEAD;
    if((frameLen > 255) || (frameLen > dstBufSz)) {
        ESP_LOGW(TAG, "payload too big to fit!");
        return -1;
    }

    /* SOF */
    dstBuf[idx++] = TAG_SOF;
    checksum = TAG_SOF;
    /* LEN */
    dstBuf[idx++] = frameLen;
    checksum += frameLen;
    /* Timestamp */
    timestamp = xTaskGetTickCount();
    dstBuf[idx] = timestamp & 0x000000FF;
    checksum += dstBuf[idx++];
    dstBuf[idx] = (timestamp >> 8) & 0x000000FF;
    checksum += dstBuf[idx++];
    dstBuf[idx] = (timestamp >> 16) & 0x000000FF;
    checksum += dstBuf[idx++];
    dstBuf[idx] = (timestamp >> 24) & 0x000000FF;
    checksum += dstBuf[idx++];
    /* Packet SEQ */
    dstBuf[idx] = sppTxSeq & 0x00FF;
    checksum += dstBuf[idx++];
    dstBuf[idx] = (sppTxSeq >> 8) & 0x00FF;
    checksum += dstBuf[idx++];
    sppTxSeq++;
    /* Payload */
    for (uint32_t i = 0; i < payloadLen; i++) {
        dstBuf[idx] = payload[i];
        checksum += dstBuf[idx++];
    }
    /* Checksum */
    dstBuf[idx++] = -checksum;

    return(idx);
}


void parse_frame(dev_buffer_t * pBuf)
{
    uint8_t checksum = 0;
    uint32_t rdPtr = 0;
    uint8_t len = 0;

    if(NULL == pBuf) {
        return;
    }

    if(pBuf->len < FRAME_RX_OVERHEAD) {
        return;
    }

    while(rdPtr < pBuf->len) {
        /* Check for start TAG */
        if(TAG_SOF != pBuf->u8Element[rdPtr]) {
            rdPtr++;
            continue;
        }
        len = pBuf->u8Element[rdPtr + LEN_OFFSET];
        checksum = 0;
        for(int i = 0; i < len; i++) {
            checksum += pBuf->u8Element[rdPtr + i];
        }

        if(0 != checksum) {
            rdPtr++;
            continue;
        } else {
            /* Valid Frame */
            if(validFrameCb != NULL) {
                validFrameCb(&pBuf->u8Element[rdPtr + FRAME_RX_PAYLOAD_CMD_OFFSET], len - FRAME_RX_OVERHEAD);
            }
            rdPtr += len;
        }
    }
}
