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
 *    Type       : 1 byte
 *    Payload    : N bytes
 *    Checksum   : 1 byte
 */
#define TAG_SOF                 (0xff)  //!< The value of the tag byte for a command packet.
#define LEN_OFFSET              (0x01)
#define TIMESTAMP_OFFSET        (0x02)
#define PACKET_SEQ_OFFSET       (0x06)
#define PAYLOAD_OFFSET          (0x08)
#define FRAME_OVERHEAD          (9)     /* 1 SOF + 1 LEN + 4 TIME + 2 SEQ + 1 CHKSM */

#define FRAME_RX_OVERHEAD               (4)  /* 1 SOF + 1 LEN + 1 TYPE + 1 CHKSM */
#define FRAME_RX_LEN_OFFSET             (1)
#define FRAME_RX_PAYLOAD_CMD_OFFSET     (3)
#define FRAME_RX_PAYLOAD_PARAM_OFFSET   (6)

void frame_parser_store(uint8_t * pBuf, uint32_t len);
void frame_parser_process();

#endif /* _FRAME_PARSER_H_ */