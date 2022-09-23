#ifndef __DATA_H__
#define __DATA_H__

#define DEVICE_BUFFER_LENGTH                (512)
#define CAN_STANDARD_BUFFER_LENGTH          (8)

typedef struct {
    int len;
    uint8_t u8Element[DEVICE_BUFFER_LENGTH];
} dev_buffer_t;

typedef struct {
    uint32_t identifier;
    uint8_t dlc;
    uint8_t data[CAN_STANDARD_BUFFER_LENGTH];
} can_buffer_t;

#endif /* __DATA_H__ */
