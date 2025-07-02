#include "network_connection/src/network_connection.h"
#include "esp_netif_ip_addr.h" // Add this include for esp_netif_ip_info_t

const char* WIFI_TAG = "[WIFI]"; // Tag for WIFI logging


// static esp_netif_t *netif;
// static EventGroupHandle_t wifi_event_group;
// static esp_event_handler_instance_t wifi_handler_event_instance;
// static esp_event_handler_instance_t got_ip_event_instance;
// int socket_desc = 0;
// static int socket_connect_attempt_count = 0;
// uint8_t tx_buffer[BUFFER_SIZE];
// uint8_t rx_buffer[BUFFER_SIZE];

static NETCONN_t netcon;

esp_err_t NETCONN_TcpServerConnect(const char *ip_string, uint16_t port)
{
  netcon.socket_descriptor = 0;
  struct sockaddr_in server_info = {0};

  server_info.sin_family = AF_INET;
  inet_aton(ip_string, &server_info.sin_addr.s_addr);
  server_info.sin_port = htons(port);

  netcon.socket_descriptor = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  ESP_LOGI(WIFI_TAG, "Created socket %d\n", netcon.socket_descriptor);
  if (netcon.socket_descriptor < 0)
  {
    ESP_LOGE(WIFI_TAG, "Failed to create socket\n");
    return TCP_FAILURE;
  }

  esp_netif_t *netif = NULL;
  netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"); // Get handle for the Wi-Fi station interface

  esp_netif_ip_info_t ip_info;
  memset(&ip_info, 0, sizeof(ip_info));
  esp_err_t ret = esp_netif_get_ip_info(netif, &ip_info);

  int optval = 1;
  int status = connect(netcon.socket_descriptor, 
                       (struct sockaddr *)&server_info,
                       sizeof(server_info));
  
                       
  if (status != 0)
  {
  ESP_LOGE(WIFI_TAG, "Failed to connect socket %s", inet_ntoa(server_info.sin_addr.s_addr));
  close(netcon.socket_descriptor);
  return TCP_FAILURE;
  }

  status = setsockopt(netcon.socket_descriptor, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
  
  if (status != 0)
  {
    ESP_LOGE(WIFI_TAG, "Failed to set option %d", status);
    close(netcon.socket_descriptor);
    return TCP_FAILURE;
  }

  netcon.self_addr.sin_family = AF_INET;
  netcon.self_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  netcon.self_addr.sin_port = htons(9009);

  status = bind(netcon.socket_descriptor, (struct sockaddr*)&netcon.self_addr, sizeof(netcon.self_addr));

  ESP_LOGI(WIFI_TAG, "Connected to TCP server\n");
  bzero(netcon.rx_buffer, sizeof(netcon.rx_buffer));
  bzero(netcon.tx_buffer, sizeof(netcon.tx_buffer));

  return TCP_SUCCESS;
}


void NETCONN_TcpServerConnectionClose(void)
{
  close(netcon.socket_descriptor);
}

int NETCONN_SendData(NETCONN_packet_bytes_t* packet)
{
  int status = 0;

  memcpy(&netcon.tx_buffer[0], 
         &packet->payload_len,
         sizeof(packet->payload_len));
  memcpy(&netcon.tx_buffer[HEADER_SIZE], 
         &packet->bytes,
         packet->payload_len);

  ESP_LOGI(WIFI_TAG,"send bytes ...\n");
  ESP_LOG_BUFFER_HEXDUMP(WIFI_TAG,
                         &netcon.tx_buffer[0],
                         packet->payload_len + HEADER_SIZE, 
                         ESP_LOG_INFO);

  status = send(netcon.socket_descriptor,
                &netcon.tx_buffer[0],
                packet->payload_len + HEADER_SIZE,
                0);
  vTaskDelay(pdMS_TO_TICKS(10));
  ESP_LOGE("[WIFI]","STATUS %d\n", status);

  return status;
}

void NETCONN_WifiEventHandler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    ESP_LOGI(WIFI_TAG, "Connectiong to AP ...");
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    if (netcon.socket_connect_attempt_count < MAX_FAILURES)
    {
      ESP_LOGI(WIFI_TAG, "Reconnectiong to AP ...");
      esp_wifi_connect();
      netcon.socket_connect_attempt_count++;
    }
    else
    {
      xEventGroupSetBits(netcon.wifi_event_group, WIFI_FAILURE);
    }
  }
}

void NETCONN_GotIpEventHandler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
  if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(WIFI_TAG, "STA IP: " IPSTR, IP2STR(&event->ip_info.ip));
    netcon.socket_connect_attempt_count = 0;
    xEventGroupSetBits(netcon.wifi_event_group, WIFI_SUCCESS);
  }
}

int NETCONN_ConnectWifi(void)
{
  int state = EXIT_FAILURE;

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));
  netcon.wifi_event_group = xEventGroupCreate();

  // esp_event_handler_t ;
  esp_event_handler_instance_register(WIFI_EVENT,
                                      ESP_EVENT_ANY_ID,
                                      NETCONN_WifiEventHandler,
                                      NULL,
                                      &(netcon.wifi_handler_event_instance));

  esp_event_handler_instance_register(IP_EVENT,
                                      IP_EVENT_STA_GOT_IP,
                                      NETCONN_GotIpEventHandler,
                                      NULL,
                                      &(netcon.got_ip_event_instance));

  /* START WIFI DRIVER */
  wifi_config_t wifi_configuration = {
      .sta = {
          .ssid = WIFI_SSID,
          .password = WIFI_KEY,
          .threshold.authmode = WIFI_AUTH_WPA2_PSK,
          .pmf_cfg = {
              .capable = true,
              .required = false
          },
      },
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_configuration));

  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(WIFI_TAG, "STA Initialization complete");

  EventBits_t bits = xEventGroupWaitBits(netcon.wifi_event_group,///  nullified but why?
                                         WIFI_SUCCESS | WIFI_FAILURE,
                                         pdFALSE,
                                         pdFALSE,
                                         portMAX_DELAY);

  if (bits & WIFI_SUCCESS)
  {
    ESP_LOGI(WIFI_TAG, "Connected to AP");
  }
  else if (bits % WIFI_FAILURE)
  {
    ESP_LOGI(WIFI_TAG, "Failed to connect to AP");
  }
  else
  {
    ESP_LOGI(WIFI_TAG, "UNEXPECTED EVENT");
  }

  esp_event_handler_instance_unregister(WIFI_EVENT,
                                        ESP_EVENT_ANY_ID,
                                        &(netcon.wifi_handler_event_instance));

  esp_event_handler_instance_unregister(IP_EVENT,
                                        IP_EVENT_STA_GOT_IP,
                                        &(netcon.got_ip_event_instance));
  vEventGroupDelete(netcon.wifi_event_group);

  vTaskDelay(pdMS_TO_TICKS(5000));
  return state;
}

