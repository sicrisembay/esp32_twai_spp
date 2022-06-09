#include "stdint.h"
#include "stddef.h"
#include "frame_parser.h"

#define FRAME_RX_SZ_BIT     (10)
#define FRAME_RX_SIZE       (1UL << FRAME_RX_SZ_BIT)
#define FRAME_RX_SZ_MASK    (FRAME_RX_SIZE - 1)

volatile uint32_t rdPtr = 0;
volatile uint32_t wrPtr = 0;
static uint8_t rxFrameBuffer[FRAME_RX_SIZE];

void frame_parser_store(uint8_t * pBuf, uint32_t len)
{
    if((pBuf != NULL) && (len > 0)) {
        for(uint32_t i = 0; i < len; i++) {
            rxFrameBuffer[wrPtr] = pBuf[i];
            wrPtr = (wrPtr + 1) & FRAME_RX_SZ_MASK;
        }
    }
}

void frame_parser_process()
{
    uint32_t availableBytes = 0;
    uint8_t length = 0;
    uint32_t idx = 0;
    uint8_t sum = 0;

    while(wrPtr != rdPtr) {
        /* Check Start of Frame */
        if (TAG_SOF != rxFrameBuffer[rdPtr]) {
            /* Not SOF, skip character */
            rdPtr = (rdPtr + 1) & FRAME_RX_SZ_MASK;
            continue;
        }

        /* Get available bytes in the buffer */
        if (wrPtr >= rdPtr) {
            availableBytes = wrPtr - rdPtr;
        } else {
            availableBytes = (wrPtr + FRAME_RX_SIZE) - rdPtr;
        }
        if (availableBytes < 2) {
            /*
             * Minimum is 2 bytes to proceed
             * 1 SOF + 1 LEN
             */
            break;
        }
        /*
         * Check if packet size is valid
         */
        length = rxFrameBuffer[(rdPtr + FRAME_RX_LEN_OFFSET) & FRAME_RX_SZ_MASK];
        if ((length < 4) || (length > 127)) {
            /*
             * Packet length minimum is 4
             * 1 SOF + 1 LEN + 1 TYPE + 1 CHKSM
             * 
             * Packet length maximum is 127
             */
            rdPtr = (rdPtr + 1) & FRAME_RX_SZ_MASK;
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
            sum += rxFrameBuffer[(rdPtr + idx) & FRAME_RX_SZ_MASK];
        }
        /*
         * Skip this packet if the checksum is not correct.
         * It is probably not really the start of packet.
         */
        if (sum != 0) {
            /* Skip this character */
            rdPtr = (rdPtr + 1) & FRAME_RX_SZ_MASK;
            /* Keep scanning for SOF */
            continue;
        }
        /*
         * It is a valid frame.
         * Process frame
         */
        /// TODO

        /*
         * Done with processing this packet
         */
        rdPtr = (rdPtr + length) & FRAME_RX_SZ_MASK;
    }
}