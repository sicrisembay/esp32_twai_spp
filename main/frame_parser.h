#ifndef _FRAME_PARSER_H_
#define _FRAME_PARSER_H_

/*
 * Frame Format (Device --> PC)
 *    TAG        : 1 byte
 *    Length     : 1 byte
 *    Timestamp  : 4 bytes
 *    Packet SEQ : 2 bytes
 *    Payload    : N bytes
 *    Checksum   : 1 byte
 * 
 * Frame Format (Device <-- PC)
 *    TAG        : 1 byte
 *    Length     : 1 byte
 *    Payload    : N bytes
 *    Checksum   : 1 byte
 * 
 *  Payload Format
 *    Offset 0  : Command/Type
 *    Offset 1  : Start of parameter (dependent on Command/Type)
 *
 *  Payload Format for CMD_SEND_DOWNSTREAM and CMD_SEND_UPSTREAM
 *    Offset 0    : CMD_SEND_DOWNSTREAM or CMD_SEND_UPSTREAM
 *    Offset 1:2  : 11-bit Message ID (little endian)
 *    Offset 3    : Payload Length (Max 8)
 *    Offset 4:11 : TWAI message (0-8 bytes)
 */
#define TAG_SOF                 (0xff)  //!< The value of the tag byte for a command packet.
#define LEN_OFFSET              (0x01)
#define TIMESTAMP_OFFSET        (0x02)
#define PACKET_SEQ_OFFSET       (0x06)
#define PAYLOAD_OFFSET          (0x08)
#define FRAME_TX_OVERHEAD       (9)     /* 1 SOF + 1 LEN + 4 TIME + 2 SEQ + 1 CHKSM */

#define FRAME_RX_OVERHEAD               (3)  /* 1 SOF + 1 LEN + 1 CHKSM */
#define FRAME_RX_LEN_OFFSET             (1)
#define FRAME_RX_PAYLOAD_CMD_OFFSET     (2)
#define FRAME_RX_PAYLOAD_PARAM_OFFSET   (3)

#define CMD_GET_TWAI_NODE_ID    (0x00)
#define CMD_SET_TWAI_BAUDRATE   (0x01)
#define CMD_ENABLE_TWAI         (0x02)
#define CMD_DISABLE_TWAI        (0x03)
#define CMD_GET_DEVICE_NAME     (0x04)

#define CMD_SEND_DOWNSTREAM     (0x10)  /* Send down to TWAI */
#define CMD_SEND_UPSTREAM       (0x11)  /* Send up to BT SPP */

typedef void (*pfn_valid_frame_cb)(uint8_t * pFrame, uint32_t len);

void parser_init(pfn_valid_frame_cb cb);
esp_err_t spp_tx_get_block(uint8_t * pBlock, uint32_t * pSize);
esp_err_t spp_send(uint8_t * pBuf, uint32_t len);
int format_frame(uint8_t * dstBuf, size_t dstBufSz, uint8_t * payload, uint32_t payloadLen);
void parse_frame(dev_buffer_t * pbuf);
//esp_err_t spp_get_block(uint8_t * pBuf, uint32_t *pLen);
void parser_store(uint8_t * pBuf, uint32_t len);
void spp_parser_process();

#endif /* _FRAME_PARSER_H_ */