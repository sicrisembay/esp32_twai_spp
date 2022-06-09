#ifndef _BT_BRIDGE_H_
#define _BT_BRIDGE_H_

#include "freertos/ringbuf.h"

typedef void (* bt_bridge_srv_open_cb_t)(void);
typedef void (* bt_bridge_write_done_cb_t)(bool bCongest);

extern void bt_bridge_init(void);
extern void bt_bridge_register_srv_open_cb(bt_bridge_srv_open_cb_t callback);
extern void bt_bridge_register_write_done_cb(bt_bridge_write_done_cb_t callback);
extern RingbufHandle_t bt_bridge_get_rx_hdl(void);
extern char * bt_bridge_get_name(void);
extern bool bt_bridge_rdy(void);
extern bool bt_bride_is_congested(void);
extern esp_err_t bt_bridge_send(uint8_t * pData, uint16_t size);

#endif /* _BT_BRIDGE_H_ */