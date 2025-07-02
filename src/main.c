#include "assert.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "drivers/ads1115/ads1115.h"
#include "network_connection/src/network_connection.h"
// #include "adc_measurement/src/adc_measurement.h"
// #include "voltage_monitor/src/voltage_monitor.h"

#define ESPBOARD_BLINK_PIN GPIO_NUM_2

#define ESPBOARD_ISR_PIN GPIO_NUM_16

#define ESPBOARD_I2C_SCL_IO GPIO_NUM_22 // I2C SCL pin
#define ESPBOARD_I2C_SDA_IO GPIO_NUM_21 // I2C SDA pin
#define ESPBOARD_I2C_FREQ_HZ 100000u    // I2C frequency in Hz
#define ESPBOARD_I2C_PORT I2C_NUM_0     // I2C port number, can be I2C_NUM_0 or I2C_NUM_1

#define SLAVE_ADDR 0x49          // I2C slave address 0x48 (GND),0x49 (SDA),0x4A (SCL),0x4B
#define SLAVE_HI_THRESH_REG 0x03 // High threshold register address

#define ADC_MEASUREMENT_PERION_MS 600U


static const char *GPIO_TAG = "[GPIO]"; // Tag for GPIO logging
static const char *I2C_TAG = "[I2C ]";  // Tag for I2C logging
static const char *ADC_TAG = "[ADC ]";  // Tag for ADC logging

static ads1115_t ads1115_handle;

void ESPBOARD_Configure(void);

void ESPBOARD_Configure(void)
{
  /* GPIO Configurations*/
  gpio_config_t blink_config =
  {
    .pin_bit_mask = (1ULL << (uint64_t)ESPBOARD_BLINK_PIN),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t err = ESP_OK;
  // cppcheck-suppress-begin[misra-c2012-17.7, misra-c2012-10.4, misra-c2012-1.2] suppressing violation related to log functions

  /* Reset pin config */
  err = gpio_reset_pin(ESPBOARD_BLINK_PIN);
  if (err != ESP_OK)
  {
    ESP_DRAM_LOGE(GPIO_TAG, "Failed to reset GPIO pin %d: %s\n", ESPBOARD_BLINK_PIN, esp_err_to_name(err));
    assert(err == ESP_OK);
  }
  err = gpio_reset_pin(ESPBOARD_ISR_PIN);
  if (err != ESP_OK)
  {
    ESP_DRAM_LOGE(GPIO_TAG, "Failed to reset GPIO pin %d: %s\n", ESPBOARD_ISR_PIN, esp_err_to_name(err));
    assert(err == ESP_OK);
  }

  /* Configure pins directions */
  err = gpio_config(&blink_config);
  if (err != ESP_OK)
  {
    ESP_DRAM_LOGE(GPIO_TAG, "Failed to configure GPIO pin %d as output: %s\n", ESPBOARD_BLINK_PIN, esp_err_to_name(err));
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
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL; // Use default clock flags, can be adjusted if needed

  err = i2c_param_config(ESPBOARD_I2C_PORT, &conf);
  if (err != ESP_OK)
  {
    ESP_DRAM_LOGE(I2C_TAG, "Failed to configure I2C bus: %s\n", esp_err_to_name(err));
    assert(err == ESP_OK);
  }
  err = i2c_driver_install(ESPBOARD_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
  if (err != ESP_OK)
  {
    ESP_DRAM_LOGE(I2C_TAG, "Failed to install I2C driver: %s\n", esp_err_to_name(err));
    assert(err == ESP_OK);
  }
  ads1115_handle = ads1115_config(ESPBOARD_I2C_PORT, SLAVE_ADDR);
  ads1115_set_mode(&ads1115_handle, ADS1115_MODE_SINGLE); // Set the ADS1115 to continuous mode
  ads1115_set_sps(&ads1115_handle, ADS1115_SPS_32);
  // cppcheck-suppress-end[misra-c2012-17.7, misra-c2012-10.4, misra-c2012-1.2] suppressing violation related to log functions
  /* Setup ESP32 network interface */
  int status = NETCONN_ConnectWifi();
  if (status != WIFI_SUCCESS)
  {
    ESP_LOGE("[WIFI]", "Failed to connect ot AP.");
  }
}

// cppcheck-suppress-begin[unusedFunction, misra-c2012-8.4] suppressing violation related to log functions
void app_main(void)
{ // cppcheck-suppress-end[unusedFunction, misra-c2012-8.4] suppressing violation related to log functions
  ESPBOARD_Configure();
  vTaskDelay(pdMS_TO_TICKS((1U)));
  TickType_t main_tick = xTaskGetTickCount();
  TickType_t i2c_request_tick = xTaskGetTickCount();
  bool led_gpio_on = false;
  NETCONN_TcpServerConnect("192.168.1.2", 9009u);

  while (1)
  {
    TickType_t current_tick = xTaskGetTickCount();
    if ((uint32_t)(pdTICKS_TO_MS(current_tick - main_tick)) >= (uint32_t)500u)
    {
      main_tick = current_tick;
      led_gpio_on = !led_gpio_on;
      (void)gpio_set_level(ESPBOARD_BLINK_PIN, !!led_gpio_on);
      esp_log_write(ESP_LOG_INFO, GPIO_TAG, "GPIO pin %d is now %s.\n", ESPBOARD_BLINK_PIN, led_gpio_on ? "ON" : "OFF");
    }
    if ((uint32_t)pdTICKS_TO_MS(current_tick - i2c_request_tick) >= (uint32_t)ADC_MEASUREMENT_PERION_MS)
    {
      i2c_request_tick = current_tick;
      int voltage_mV = 0.0;                                 // Variable to hold ADC value
      ads1115_set_sps(&ads1115_handle, ADS1115_SPS_8);        // Set the sampling rate to 64 SPS
      ads1115_set_mode(&ads1115_handle, ADS1115_MODE_SINGLE); // Set the ADS1115 to continuous mode
      ads1115_set_mux(&ads1115_handle, ADS1115_MUX_0_GND);    // Set the multiplexer to read from channel 0 (AIN0)
      voltage_mV = (int)(ads1115_get_voltage(&ads1115_handle) * 1000.0f);       // Read ADC value from ADS1115
      esp_log_write(ESP_LOG_INFO, ADC_TAG, "value %d at tick %ld\n", voltage_mV, i2c_request_tick);
      
      
      NETCONN_packet_bytes_t packet;
      packet.payload_len = 5;
      memcpy(&packet.bytes[0], (void*)&voltage_mV, packet.payload_len);
      NETCONN_SendData(&packet);
    }
  }
  NETCONN_TcpServerConnectionClose();
}
