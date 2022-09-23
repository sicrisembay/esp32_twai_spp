#ifndef CAN_H
#define CAN_H

void can_init(QueueHandle_t * canTxQueue, QueueHandle_t * canRxQueue);

#endif /* CAN_H */
