#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <inttypes.h>


#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"


#include "drivers/ads1115/ads1115.h"
// #include "adc_measurement/src/adc_measurement.h"
// #include "voltage_monitor/src/voltage_monitor.h"

#define WIFI_SUCCESS (1 << 0)
#define WIFI_FAILURE (1 << 1)
#define TCP_SUCCESS (1 << 0)
#define TCP_FAILURE (1 << 1)
#define MAX_FAILURES 10

#define WIFI_SSID "Kyivstar-3BC4"
#define WIFI_KEY  "password"
#define BUFFER_SIZE 256

#define ESPBOARD_BLINK_PIN      GPIO_NUM_2

#define ESPBOARD_ISR_PIN        GPIO_NUM_16

#define ESPBOARD_I2C_SCL_IO	    GPIO_NUM_22	    // I2C SCL pin
#define ESPBOARD_I2C_SDA_IO	    GPIO_NUM_21	    // I2C SDA pin
#define ESPBOARD_I2C_FREQ_HZ    100000u          // I2C frequency in Hz
#define ESPBOARD_I2C_PORT       I2C_NUM_0       // I2C port number, can be I2C_NUM_0 or I2C_NUM_1

#define SLAVE_ADDR              0x49            // I2C slave address 0x48 (GND),0x49 (SDA),0x4A (SCL),0x4B 
#define SLAVE_HI_THRESH_REG     0x03             // High threshold register address

#define ADC_MEASUREMENT_PERION_MS 600U

#include "assert.h"

static const char* GPIO_TAG =   "[GPIO]";       // Tag for GPIO logging
static const char* I2C_TAG =    "[I2C ]";       // Tag for I2C logging
static const char* ADC_TAG =    "[ADC ]";       // Tag for ADC logging
static const char* WIFI_TAG =   "[WIFI]";       // Tag for WIFI logging


static ads1115_t ads1115_handle;
static esp_netif_t* netif;
static EventGroupHandle_t wifi_event_group;
static esp_event_handler_t  wifi_event_handler_;
static esp_event_handler_instance_t wifi_handler_event_instance;
static esp_event_handler_instance_t got_ip_event_instance;
int socket_desc = 0;
static int s_retry_num = 0;
uint8_t tx_buffer[BUFFER_SIZE];
uint8_t rx_buffer[BUFFER_SIZE];

void ESPBOARD_Configure(void);
int ESPBOARD_ConnectWifi(void);
esp_err_t tcp_server_connect(int* socket_descriptor);

void set_wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);

void got_ip_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);

esp_err_t tcp_server_connect(int* socket_descriptor)
{
  *socket_descriptor = 0;
  struct sockaddr_in serverInfo = {0};
  char readBuffer[1024] = {0};
  const char* ip_string = "192.168.1.24";

  serverInfo.sin_family = AF_INET;
  // serverInfo.sin_len
  
  inet_aton(ip_string, &serverInfo.sin_addr.s_addr);//0x1801A8C0;          // 192.168.1.24
  // inet_
  serverInfo.sin_port = htons(9888);

  (*socket_descriptor) = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
  ESP_LOGI(WIFI_TAG,"Created socket %d\n", (*socket_descriptor));
  if((*socket_descriptor) < 0)
  {
    ESP_LOGE(WIFI_TAG,"Failed to create socket\n");
    return TCP_FAILURE;
  }

  int status = connect((*socket_descriptor), (struct sockaddr*)&serverInfo, sizeof(serverInfo));
  if(status != 0)
  {
    ESP_LOGE(WIFI_TAG,"Failed to connect socket %s", inet_ntoa(serverInfo.sin_addr.s_addr));
    close((*socket_descriptor));
    return TCP_FAILURE;
  }

  ESP_LOGI(WIFI_TAG,"Connected to TCP server\n");
  bzero(readBuffer,sizeof(readBuffer));
  int r = read((*socket_descriptor), readBuffer, sizeof(readBuffer) - 1);
  for (int i = 0; i < r; i++)
  {
    putchar(readBuffer[i]);
  }
  
  return TCP_SUCCESS;
}

void set_wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    ESP_LOGI(WIFI_TAG,"Connectiong to AP ...");
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    if(s_retry_num < MAX_FAILURES)
    {
      ESP_LOGI(WIFI_TAG, "Reconnectiong to AP ...");
      esp_wifi_connect();
      s_retry_num++;
    }
    else
    {
      xEventGroupSetBits(wifi_event_group, WIFI_FAILURE);
    }
  }
}

void got_ip_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
  if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
    ESP_LOGI(WIFI_TAG,"STA IP: " IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(wifi_event_group, WIFI_SUCCESS);
  }
}

int ESPBOARD_ConnectWifi(void) 
{
  int state = EXIT_FAILURE;


  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  netif = esp_netif_create_default_wifi_sta();

  wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));
  wifi_event_group = xEventGroupCreate();
  
  // esp_event_handler_t ;
  esp_event_handler_instance_register(WIFI_EVENT,
                                      ESP_EVENT_ANY_ID,
                                      set_wifi_event_handler,
                                      NULL,
                                      &wifi_handler_event_instance);

  esp_event_handler_instance_register(IP_EVENT,
                                      IP_EVENT_STA_GOT_IP,
                                      got_ip_event_handler,
                                      NULL,
                                      &got_ip_event_instance);

  /* START WIFI DRIVER */
  wifi_config_t wifi_configuration ={
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

  EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                         WIFI_SUCCESS | WIFI_FAILURE,
                                         pdFALSE,
                                         pdFALSE,
                                         portMAX_DELAY);

  if(bits & WIFI_SUCCESS)
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
                                      &wifi_handler_event_instance);

  esp_event_handler_instance_unregister(IP_EVENT,
                                      IP_EVENT_STA_GOT_IP,
                                      &got_ip_event_instance);
  vEventGroupDelete(wifi_event_group);

  return state;
}

void ESPBOARD_Configure(void)
{
  /* GPIO Configurations*/ 
  gpio_config_t blink_config = 
  {
    .pin_bit_mask = (1ULL << (uint64_t)ESPBOARD_BLINK_PIN),
    .mode         = GPIO_MODE_OUTPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE,
  };

  esp_err_t err = ESP_OK;
                                                                                                    // cppcheck-suppress-begin[misra-c2012-17.7, misra-c2012-10.4, misra-c2012-1.2] suppressing violation related to log functions

  /* Reset pin config */
  err = gpio_reset_pin(ESPBOARD_BLINK_PIN);
  if (err != ESP_OK) {
    ESP_DRAM_LOGE( GPIO_TAG, "Failed to reset GPIO pin %d: %s\n", ESPBOARD_BLINK_PIN, esp_err_to_name(err));
    assert(err == ESP_OK);
  }
  err = gpio_reset_pin(ESPBOARD_ISR_PIN);
  if (err != ESP_OK) {
    ESP_DRAM_LOGE( GPIO_TAG, "Failed to reset GPIO pin %d: %s\n", ESPBOARD_ISR_PIN, esp_err_to_name(err));
    assert(err == ESP_OK);
  }
  
  
  /* Configure pins directions */
  err = gpio_config(&blink_config);
  if (err != ESP_OK) {
    ESP_DRAM_LOGE( GPIO_TAG, "Failed to configure GPIO pin %d as output: %s\n", ESPBOARD_BLINK_PIN, esp_err_to_name(err));
    assert(err == ESP_OK);
  }
  
  /* Initialize I2C bus */
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = ESPBOARD_I2C_SDA_IO;
  conf.scl_io_num = ESPBOARD_I2C_SCL_IO;
  conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
  conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
  conf.master.clk_speed = ESPBOARD_I2C_FREQ_HZ;
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;                                                     // Use default clock flags, can be adjusted if needed

  err = i2c_param_config(ESPBOARD_I2C_PORT, &conf);
  if (err != ESP_OK) {
    ESP_DRAM_LOGE(I2C_TAG, "Failed to configure I2C bus: %s\n", esp_err_to_name(err));
    assert(err == ESP_OK);
  }
  err = i2c_driver_install(ESPBOARD_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
  if (err != ESP_OK) {
    ESP_DRAM_LOGE(I2C_TAG, "Failed to install I2C driver: %s\n", esp_err_to_name(err));
    assert(err == ESP_OK);
  }
  ads1115_handle = ads1115_config(ESPBOARD_I2C_PORT,SLAVE_ADDR);
  ads1115_set_mode(&ads1115_handle, ADS1115_MODE_CONTINUOUS);                                       // Set the ADS1115 to continuous mode
  ads1115_set_sps(&ads1115_handle, ADS1115_SPS_32);
                                                                                                    // cppcheck-suppress-end[misra-c2012-17.7, misra-c2012-10.4, misra-c2012-1.2] suppressing violation related to log functions
  /* Setup ESP32 network interface */
  int status = ESPBOARD_ConnectWifi();
  if (status != WIFI_SUCCESS)
  {
    ESP_LOGE(WIFI_TAG,"Failed to connect ot AP.");
  }
}

                                                                                                    // cppcheck-suppress-begin[unusedFunction, misra-c2012-8.4] suppressing violation related to log functions
void app_main(void)
{                                                                                                   // cppcheck-suppress-end[unusedFunction, misra-c2012-8.4] suppressing violation related to log functions
  ESPBOARD_Configure();
  vTaskDelay(pdMS_TO_TICKS((1U)));
  TickType_t main_tick = xTaskGetTickCount();
  TickType_t i2c_request_tick = xTaskGetTickCount();
  bool led_gpio_on = false;
  tcp_server_connect(&socket_desc);

  while (1) 
  {
    TickType_t current_tick = xTaskGetTickCount();
    if ((uint32_t)(pdTICKS_TO_MS(current_tick - main_tick)) >= (uint32_t)500u) {
      main_tick = current_tick;
      led_gpio_on = !led_gpio_on;
      (void)gpio_set_level(ESPBOARD_BLINK_PIN, !!led_gpio_on);
      esp_log_write(ESP_LOG_INFO, GPIO_TAG, "GPIO pin %d is now %s.\n", ESPBOARD_BLINK_PIN, led_gpio_on ? "ON" : "OFF");
    }
    if ((uint32_t)pdTICKS_TO_MS(current_tick - i2c_request_tick) >= (uint32_t)ADC_MEASUREMENT_PERION_MS)
    {
      i2c_request_tick = current_tick;
      double adc_value = 0.0;                                                                     // Variable to hold ADC value
      ads1115_set_sps(&ads1115_handle, ADS1115_SPS_8);                                            // Set the sampling rate to 64 SPS
      ads1115_set_mode(&ads1115_handle, ADS1115_MODE_SINGLE);                                     // Set the ADS1115 to continuous mode
      ads1115_set_mux(&ads1115_handle, ADS1115_MUX_0_GND);                                        // Set the multiplexer to read from channel 0 (AIN0)
      adc_value = ads1115_get_voltage(&ads1115_handle);                                           // Read ADC value from ADS1115
      esp_log_write(ESP_LOG_INFO, ADC_TAG, "value %lf at tick %ld\n", adc_value, i2c_request_tick);
      memcpy(&tx_buffer[0], &adc_value, sizeof(double));
      write(socket_desc, &tx_buffer[0], sizeof(double));
      // close(socket_desc);
    }
  }
}
