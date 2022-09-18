#ifndef TCP_SERVER_H
#define TCP_SERVER_H

void tcp_server_init(uint32_t port, QueueHandle_t *txQueue, QueueHandle_t *rxQueue);
bool tcp_server_socket_connected(void);

#endif /* TCP_SERVER_H */
