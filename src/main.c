#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "voltage_monitor.h"

#include "esp_log.h"

#define BLINK_GPIO GPIO_NUM_2

void Modules_Init(){
    VM_Init();
}

void app_main() 
{
    
    esp_rom_gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    Modules_Init();
    VM_ScanForChannelsAndPorts();
    vTaskDelay((VOLTAGE_MONITOR_MEASURE_PERIOD_MS * 2) / portTICK_PERIOD_MS);

    while (1) 
    {
        gpio_set_level(BLINK_GPIO, 1);
        esp_log_write(ESP_LOG_INFO,"*","LED ON\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        esp_log_write(ESP_LOG_INFO,"*","LED OFF\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);

        VM_Update();
        vTaskDelay(VOLTAGE_MONITOR_MEASURE_PERIOD_MS / portTICK_PERIOD_MS);
    }
}
