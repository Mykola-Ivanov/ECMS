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
// #define ESPBOARD_I2C_PORT I2C_NUM_0             // I2C port number, can be I2C_NUM_0 or I2C_NUM_1
// #define ESPBOARD_I2C_MASTER_TX_BUF_DISABLE 0    // I2C master TX buffer size
// #define ESPBOARD_I2C_MASTER_RX_BUF_DISABLE 0    // I2C master RX buffer size

#define SLAVE_ADDR              0x49            // I2C slave address 0x48 (GND),0x49 (SDA),0x4A (SCL),0x4B 
#define SLAVE_COMP_POL          0x01            // Comparator polarity (0: active low, 1: active high)
#define SLAVE_COMP_LAT          0x00            // Latching comparator (0: triger and clears, 1: remain latched)
#define SLAVE_WRITE_BIT         0x00            // I2C write bit
#define SLAVE_READ_BIT          0x01            // I2C read bit
#define SLAVE_CONV_REG          0x00             // Conversion register address
#define SLAVE_CONF_REG          0x01             // Configuration register address
#define SLAVE_LO_THRESH_REG     0x02             // Low threshold register address
#define SLAVE_HI_THRESH_REG     0x03             // High threshold register address

#include "assert.h"

volatile int interrupt_counter = 0;


static const char* GPIO_TAG =   "[GPIO]";       // Tag for GPIO logging
static const char* I2C_TAG =    "[I2C ]";       // Tag for I2C logging
static const char* ADC_TAG =    "[ADC ]";       // Tag for ADC logging

// i2c_master_bus_handle_t bus_handle;             // Handle for the I2C master bus
// i2c_master_dev_handle_t ads1115_handle;         // Handle for the ADS1115 device

// i2c_device_config_t dev_cfg = {
//   .dev_addr_length = I2C_ADDR_BIT_LEN_7,
//   .device_address = SLAVE_ADDR,
//   .scl_speed_hz = ESPBOARD_I2C_FREQ_HZ,
// };
ads1115_t ads1115_handle;

  



void ESPBOARD_Configure(void);

/**
 * @brief This function is the ISR handler for ADC measurement ready.
 * It disables interrupts, processes the ADC measurement, and then re-enables interrupts.
 * 
 * @param arg Pointer to any argument passed to the ISR handler (not used here).
 */
static void IRAM_ATTR ADCMEAS_AdcMeasurementReadyISRHandler(void *arg)
{
    (void)arg;
    esp_err_t err;
    /* Disable interrupts */
    // gpio_uninstall_isr_service();
    // gpio_isr_handler_remove(ESPBOARD_ISR_PIN);
    err = gpio_intr_disable(ESPBOARD_ISR_PIN);
    gpio_set_level(ESPBOARD_BLINK_PIN, 1u); // Set the blink pin high to indicate ISR entry
    if (err != ESP_OK) {
        ESP_DRAM_LOGE(GPIO_TAG, "Failed to disable interrupts for GPIO pin %d: %s\n", ESPBOARD_ISR_PIN, esp_err_to_name(err));
        return;
    }
    
    // This function will be called when the ADC measurement is ready.
    // You can implement your ISR logic here.
    // For example, you might want to read the ADC value and process it.
    ESP_DRAM_LOGI( GPIO_TAG, "ADC measurement ready interrupt triggered.\n");
    interrupt_counter++;

    /* Enable interrupts */
    // gpio_isr_handler_add(ESPBOARD_ISR_PIN, ADCMEAS_AdcMeasurementReadyISRHandler, NULL);
    gpio_intr_enable(ESPBOARD_ISR_PIN);
    if (err != ESP_OK) {
        ESP_DRAM_LOGE(GPIO_TAG, "Failed to re-enable interrupts for GPIO pin %d: %s\n", ESPBOARD_ISR_PIN, esp_err_to_name(err));
        return;
    }
    // gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
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
  /* I2C Configurations */
  // i2c_master_bus_config_t i2c_config = {
  //   .i2c_port = I2C_NUM_0, // Use I2C_NUM_0 or I2C_NUM_1 as needed
  //   .sda_io_num = ESPBOARD_I2C_SDA_IO,
  //   .scl_io_num = ESPBOARD_I2C_SCL_IO,
  //   .clk_source = I2C_CLK_SRC_DEFAULT, // Default clock source
  //   .glitch_ignore_cnt = 7, // Typical value for glitch filtering
  //   .intr_priority = 1, // Set interrupt
  //   .flags = {
  //     .enable_internal_pullup = 0, // Enable internal pull-ups
  //     .allow_pd = 0, // Allow power
  //   }
  // };

  esp_err_t err = ESP_OK;
  
  /* Reset pin config */
  err = gpio_reset_pin(ESPBOARD_BLINK_PIN);
  ESP_DRAM_LOGI( GPIO_TAG, "Reset GPIO pin %d to low. with esp_state %s\n", ESPBOARD_BLINK_PIN, esp_err_to_name(err));
  assert(err == ESP_OK);
  err = gpio_reset_pin(ESPBOARD_ISR_PIN);
  ESP_DRAM_LOGI( GPIO_TAG, "Reset GPIO pin %d to low. with esp_state %s\n", ESPBOARD_ISR_PIN, esp_err_to_name(err));
  assert(err == ESP_OK);
  
  
  /* Configure pins directions */
  err = gpio_config(&blink_config);
  ESP_DRAM_LOGI( GPIO_TAG, "Configuring GPIO pin %d as output. with esp_state %s\n", ESPBOARD_BLINK_PIN, esp_err_to_name(err));
  assert(err == ESP_OK);
  // esp_rom_gpio_pad_select_gpio(ESPBOARD_BLINK_PIN);
  err = gpio_config(&i2c_adc_interrupt_config);
  ESP_DRAM_LOGI( GPIO_TAG, "Configured GPIO pin %d as input with interrupt. with esp_state %s\n", ESPBOARD_ISR_PIN, esp_err_to_name(err));
  assert(err == ESP_OK);
  
  /* Set ISR handlers */
  err = gpio_install_isr_service(ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM);
  ESP_DRAM_LOGI( GPIO_TAG, "ISR service installed with esp_state %s\n", esp_err_to_name(err));
  assert(err == ESP_OK);
  err = gpio_isr_handler_add(ESPBOARD_ISR_PIN, ADCMEAS_AdcMeasurementReadyISRHandler, NULL);
  ESP_DRAM_LOGI( GPIO_TAG, "Added ISR handler for GPIO pin %d. with esp_state %s\n", ESPBOARD_ISR_PIN, esp_err_to_name(err));
  assert(err == ESP_OK);
  err = gpio_intr_enable(ESPBOARD_ISR_PIN);
  ESP_DRAM_LOGI( GPIO_TAG, "Enabled interrupts for GPIO pin %d. with esp_state %s\n", ESPBOARD_ISR_PIN, esp_err_to_name(err));
  assert(err == ESP_OK);  
  
  /* Initialize I2C bus */
  // err = i2c_new_master_bus(&i2c_config, &bus_handle);
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = ESPBOARD_I2C_SDA_IO;
  conf.scl_io_num = ESPBOARD_I2C_SCL_IO;
  conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
  conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
  conf.master.clk_speed = ESPBOARD_I2C_FREQ_HZ;
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL; // Use default clock flags, can be adjusted if needed

  err = i2c_param_config(ESPBOARD_I2C_PORT, &conf);
  ESP_DRAM_LOGI( I2C_TAG, "I2C bus configured with esp_state %s\n", esp_err_to_name(err));
  assert(err == ESP_OK);
  err = i2c_driver_install(ESPBOARD_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
  ESP_DRAM_LOGI( I2C_TAG, "I2C master bus initialized with esp_state %s\n", esp_err_to_name(err));
  assert(err == ESP_OK);
  ads1115_handle = ads1115_config(ESPBOARD_I2C_PORT,SLAVE_ADDR);
  ads1115_set_mode(&ads1115_handle, ADS1115_MODE_CONTINUOUS); // Set the ADS1115 to continuous mode
  ads1115_set_rdy_pin(&ads1115_handle, ESPBOARD_ISR_PIN); // Set the ready pin for ADS1115
  // ads1115_set_rdy_pin(&ads1115_handle, ESPBOARD_ISR_PIN); // Set the ready pin for ADS1115

  /* Configure the ADS1115 device */
  // uint8_t config_data[3] = {
  //   SLAVE_CONF_REG, // Configuration register address
  //   (SLAVE_COMP_POL << 7) | (SLAVE_COMP_LAT << 6) | (SLAVE_WRITE_BIT), // Configuration byte
  //   0x00 // Additional configuration byte, adjust as needed
  // };
  // err = i2c_master_transmit(ads1115_handle, config_data, sizeof(config_data), 100);
  // if (err != ESP_OK) {
  //   ESP_DRAM_LOGE(I2C_TAG, "Failed to configure ADS1115: %s\n", esp_err_to_name(err));
  // } else {
  //   ESP_DRAM_LOGI(I2C_TAG, "ADS1115 configured successfully.\n");
  // }

  /* Configure ADS1115 for continuous conversion mode */
  // uint8_t config_continuous[3] = {
  //   SLAVE_CONF_REG, // Configuration register address
  //   0xC3, // Continuous conversion mode configuration byte (example value)
  //   0x00 // Additional configuration byte, adjust as needed
  // };
  // err = i2c_master_transmit(ads1115_handle, config_continuous, sizeof(config_continuous), 100);
  // if (err != ESP_OK) {
  //   ESP_DRAM_LOGE(I2C_TAG, "Failed to set ADS1115 to continuous conversion mode: %s\n", esp_err_to_name(err));
  // } else {
  //   ESP_DRAM_LOGI(I2C_TAG, "ADS1115 set to continuous conversion mode successfully.\n");
  // }

}


void app_main(void)
{
  ESPBOARD_Configure();
  vTaskDelay(pdMS_TO_TICKS((1U)));
  TickType_t main_tick = xTaskGetTickCount();
  TickType_t i2c_request_tick = xTaskGetTickCount();
  uint32_t led_gpio_on = 0;

  while (1) 
  {
    if (pdTICKS_TO_MS(xTaskGetTickCount() - main_tick) >= (TickType_t)500) {
      main_tick = xTaskGetTickCount();
      led_gpio_on = !led_gpio_on;
      gpio_set_level(ESPBOARD_BLINK_PIN, !!led_gpio_on);
      esp_log_write(ESP_LOG_INFO, GPIO_TAG, "GPIO pin %d is now %s.\n", ESPBOARD_BLINK_PIN, led_gpio_on ? "ON" : "OFF");
      esp_log_write(ESP_LOG_INFO, I2C_TAG, "Interrupt counter: %d\n", interrupt_counter);
    }
    if (pdTICKS_TO_MS(xTaskGetTickCount() - i2c_request_tick) >= (TickType_t)2000)
    {
      esp_err_t err = ESP_OK;
      i2c_request_tick = xTaskGetTickCount();
      int32_t adc_value = 0; // Variable to hold ADC value
      // ads1115_read_adc(&adc_value);
      ads1115_set_sps(&ads1115_handle, ADS1115_SPS_8); // Set the sampling rate to 64 SPS
      ads1115_set_mode(&ads1115_handle, ADS1115_MODE_SINGLE); // Set the ADS1115 to continuous mode
      ads1115_set_mux(&ads1115_handle, ADS1115_MUX_0_GND); // Set the multiplexer to read from channel 0 (AIN0)
      // ads1115_set_rdy_pin(&ads1115_handle, ESPBOARD_ISR_PIN); // Set the ready pin for ADS1115
      // ESP_DRAM_LOGI( ADC_TAG, "ADS1115 configured for continuous mode with 64 SPS sampling rate.\n");
      // err = ads1115_write_register(&ads1115_handle, ADS1115_CONFIG_REGISTER_ADDR, ads1115_handle.config.reg);
      // if (err != ESP_OK) {
      //   ESP_DRAM_LOGE(I2C_TAG, "Failed to write configuration to ADS1115: %s\n", esp_err_to_name(err));
      //   continue; // Skip the rest of the loop if configuration fails
      // }

      adc_value = (int32_t)(1000 * ads1115_get_voltage(&ads1115_handle)); // Read ADC value from ADS1115
      ESP_DRAM_LOGI( ADC_TAG, "ADC value read: %d at tick: %d\n", adc_value, i2c_request_tick);

      
      /* Read data from the ADS1115 */
      
    }
  }
}
