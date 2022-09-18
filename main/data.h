#ifndef __DATA_H__
#define __DATA_H__

#define DEVICE_BUFFER_LENGTH        (64)

typedef struct {
    int len;
    uint8_t u8Element[DEVICE_BUFFER_LENGTH];
} dev_buffer_t;

#endif /* __DATA_H__ */
