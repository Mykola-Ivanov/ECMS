#ifndef NETCONN_H
#define NETCONN_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_err.h"
#include "esp_log.h"

#include <stdint.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/ip4_addr.h"

#include "nvs_flash.h"

#include "config/secrets.h"

#define WIFI_SUCCESS (1 << 0)
#define WIFI_FAILURE (1 << 1)
#define TCP_SUCCESS (1 << 0)
#define TCP_FAILURE (1 << 1)
#define MAX_FAILURES 10


#define BUFFER_SIZE 256
#define HEADER_SIZE 1
#define MAX_PACKET_PAYLOAD 24
#define MAX_PACKET_SIZE (HEADER_SIZE + MAX_PACKET_PAYLOAD)



typedef struct NETCONN_s {
  uint8_t                      tx_buffer[BUFFER_SIZE];
  uint8_t                      rx_buffer[BUFFER_SIZE];
  uint16_t                     socket_connect_attempt_count;// socket_connect_attempt_count;
  int                          socket_descriptor;
  EventGroupHandle_t           wifi_event_group;
  esp_event_handler_instance_t wifi_handler_event_instance;
  esp_event_handler_instance_t got_ip_event_instance;
  struct sockaddr_in           self_addr;
} NETCONN_t;

typedef struct NETCONN_packet_bytes_s
{
  uint8_t payload_len;
  uint8_t bytes[MAX_PACKET_PAYLOAD];
} NETCONN_packet_bytes_t;



int NETCONN_ConnectWifi(void);

esp_err_t NETCONN_TcpServerConnect(const char *ip_string, uint16_t port);

void NETCONN_TcpServerConnectionClose(void);

int NETCONN_SendData(NETCONN_packet_bytes_t* packet);

void NETCONN_WifiEventHandler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data);

void NETCONN_GotIpEventHandler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);


#endif //NETCONN_H
