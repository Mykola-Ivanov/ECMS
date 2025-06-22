#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "drivers/ads1115/ads1115.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include <inttypes.h>
// #include "adc_measurement/src/adc_measurement.h"
// #include "voltage_monitor/src/voltage_monitor.h"


#define ESPBOARD_BLINK_PIN      GPIO_NUM_2

#define ESPBOARD_ISR_PIN        GPIO_NUM_16

#define ESPBOARD_I2C_SCL_IO	    GPIO_NUM_22	    // I2C SCL pin
#define ESPBOARD_I2C_SDA_IO	    GPIO_NUM_21	    // I2C SDA pin
#define ESPBOARD_I2C_FREQ_HZ    100000u          // I2C frequency in Hz
#define ESPBOARD_I2C_PORT       I2C_NUM_0       // I2C port number, can be I2C_NUM_0 or I2C_NUM_1

#define SLAVE_ADDR              0x49            // I2C slave address 0x48 (GND),0x49 (SDA),0x4A (SCL),0x4B 
// #define SLAVE_COMP_POL          0x01            // Comparator polarity (0: active low, 1: active high)
// #define SLAVE_COMP_LAT          0x00            // Latching comparator (0: triger and clears, 1: remain latched)
// #define SLAVE_WRITE_BIT         0x00            // I2C write bit
// #define SLAVE_READ_BIT          0x01            // I2C read bit
// #define SLAVE_CONV_REG          0x00             // Conversion register address
// #define SLAVE_CONF_REG          0x01             // Configuration register address
// #define SLAVE_LO_THRESH_REG     0x02             // Low threshold register address
#define SLAVE_HI_THRESH_REG     0x03             // High threshold register address

#include "assert.h"


volatile int interrupt_counter = 0;

static const char* GPIO_TAG =   "[GPIO]";       // Tag for GPIO logging
static const char* I2C_TAG =    "[I2C ]";       // Tag for I2C logging
static const char* ADC_TAG =    "[ADC ]";       // Tag for ADC logging

static ads1115_t ads1115_handle;

void ESPBOARD_Configure(void);

/**
 * @brief This function is the ISR handler for ADC measurement ready.
 * It disables interrupts, processes the ADC measurement, and then re-enables interrupts.
 * 
 * @param arg Pointer to any argument passed to the ISR handler (not used here).
 */
static void IRAM_ATTR ADCMEAS_AdcMeasurementReadyISRHandler(void *arg)
{
                                                                                                    // cppcheck-suppress-begin[misra-c2012-17.7, misra-c2012-10.4, misra-c2012-1.2] suppressing violation related to log functions
  (void)arg;
  esp_err_t err;
  /* Disable interrupts */
  err = gpio_intr_disable(ESPBOARD_ISR_PIN);
  if (ESP_OK != err) {
    ESP_DRAM_LOGE(GPIO_TAG, "Failed to disable interrupts for GPIO pin %d: %s\n", ESPBOARD_ISR_PIN, esp_err_to_name(err));
  }
  err = gpio_set_level(ESPBOARD_BLINK_PIN, (uint32_t)1u); // Set the blink pin high to indicate ISR entry
  if (ESP_OK != err) {
    ESP_DRAM_LOGE(GPIO_TAG, "Failed to disable interrupts for GPIO pin %d: %s\n", ESPBOARD_ISR_PIN, esp_err_to_name(err));
  }
    
  // This function will be called when the ADC measurement is ready.
  // You can implement your ISR logic here.
  // For example, you might want to read the ADC value and process it.
  ESP_DRAM_LOGI( GPIO_TAG, "ADC measurement ready interrupt triggered.\n");
  interrupt_counter++;

  /* Enable interrupts */
  gpio_intr_enable(ESPBOARD_ISR_PIN);
  if (ESP_OK != err) {
    ESP_DRAM_LOGE(GPIO_TAG, "Failed to re-enable interrupts for GPIO pin %d: %s\n", ESPBOARD_ISR_PIN, esp_err_to_name(err));
  }
                                                                                                    // cppcheck-suppress-end[misra-c2012-17.7, misra-c2012-10.4, misra-c2012-1.2]
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

  gpio_config_t i2c_adc_interrupt_config = 
  {
    .pin_bit_mask = (1ULL << (uint64_t)ESPBOARD_ISR_PIN),
    .mode         = GPIO_MODE_INPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ONLY,
    .intr_type    = GPIO_INTR_NEGEDGE,
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
  // esp_rom_gpio_pad_select_gpio(ESPBOARD_BLINK_PIN);
  err = gpio_config(&i2c_adc_interrupt_config);
  if (err != ESP_OK) {
    ESP_DRAM_LOGE( GPIO_TAG, "Failed to configure GPIO pin %d as input: %s\n", ESPBOARD_ISR_PIN, esp_err_to_name(err));
    assert(err == ESP_OK);
  }

  /* Set ISR handlers */
  err = gpio_install_isr_service(ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM);
  if (err != ESP_OK) {
    ESP_DRAM_LOGE(GPIO_TAG, "Failed to install ISR service: %s\n", esp_err_to_name(err));
    assert(err == ESP_OK);
  }
  err = gpio_isr_handler_add(ESPBOARD_ISR_PIN, ADCMEAS_AdcMeasurementReadyISRHandler, NULL);
  if (err != ESP_OK) {
    ESP_DRAM_LOGE(GPIO_TAG, "Failed to add ISR handler for GPIO pin %d: %s\n", ESPBOARD_ISR_PIN, esp_err_to_name(err));
    assert(err == ESP_OK);
  }
  err = gpio_intr_enable(ESPBOARD_ISR_PIN);
  if (err != ESP_OK) {
    ESP_DRAM_LOGE(GPIO_TAG, "Failed to enable interrupts for GPIO pin %d: %s\n", ESPBOARD_ISR_PIN, esp_err_to_name(err));
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
  ads1115_set_rdy_pin(&ads1115_handle, ESPBOARD_ISR_PIN);                                           // Set the ready pin for ADS1115
                                                                                                    // cppcheck-suppress-end[misra-c2012-17.7, misra-c2012-10.4, misra-c2012-1.2] suppressing violation related to log functions
}

                                                                                                    // cppcheck-suppress-begin[unusedFunction, misra-c2012-8.4] suppressing violation related to log functions
void app_main(void)
{                                                                                                   // cppcheck-suppress-end[unusedFunction, misra-c2012-8.4] suppressing violation related to log functions
  ESPBOARD_Configure();
  vTaskDelay(pdMS_TO_TICKS((1U)));
  TickType_t main_tick = xTaskGetTickCount();
  TickType_t i2c_request_tick = xTaskGetTickCount();
  bool led_gpio_on = false;

  while (1) 
  {
    TickType_t current_tick = xTaskGetTickCount();
    if ((uint32_t)(pdTICKS_TO_MS(current_tick - main_tick)) >= (uint32_t)500u) {
      main_tick = xTaskGetTickCount();
      led_gpio_on = !led_gpio_on;
      (void)gpio_set_level(ESPBOARD_BLINK_PIN, !!led_gpio_on);
      esp_log_write(ESP_LOG_INFO, GPIO_TAG, "GPIO pin %d is now %s.\n", ESPBOARD_BLINK_PIN, led_gpio_on ? "ON" : "OFF");
      esp_log_write(ESP_LOG_INFO, I2C_TAG, "Interrupt counter: %d\n", interrupt_counter);
    }
    if ((uint32_t)pdTICKS_TO_MS(current_tick - i2c_request_tick) >= (uint32_t)2000)
    {
      int32_t adc_value = 0;                                                                        // Variable to hold ADC value
      ads1115_set_sps(&ads1115_handle, ADS1115_SPS_8);                                              // Set the sampling rate to 64 SPS
      ads1115_set_mode(&ads1115_handle, ADS1115_MODE_SINGLE);                                       // Set the ADS1115 to continuous mode
      ads1115_set_mux(&ads1115_handle, ADS1115_MUX_0_GND);                                          // Set the multiplexer to read from channel 0 (AIN0)
      adc_value = (int32_t)(1000 * ads1115_get_voltage(&ads1115_handle));                           // Read ADC value from ADS1115
      ESP_DRAM_LOGI(ADC_TAG, "ADC value read: %d at tick: %d\n", adc_value, i2c_request_tick);
    }
  }
}
