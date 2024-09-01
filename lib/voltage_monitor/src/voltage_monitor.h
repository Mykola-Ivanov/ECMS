
#ifndef VOLTAGE_MONITOR_H
#define VOLTAGE_MONITOR_H

// TODO: voltage monitor update
// TODO: voltage monitor get statistics 
// TODO: measure procedure
// TODO: prepare to measure
// TODO: 

#include "cfg/main_cfg.h"

#include <stdbool.h>
#include <memory.h>

#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include <driver/i2c.h>
#include "freertos/FreeRTOS.h"

#if VOLTAGE_MONITOR_USE_BUILDIN_ADC
#define VOLTAGE_MONITOR_ADC_BITNESS                             VOLTAGE_MONITOR_12BIT_ADC 

#else
#define VOLTAGE_MONITOR_ADC_BITNESS                             VOLTAGE_MONITOR_16BIT_ADC 
#endif

#define VOLTAGE_MONITOR_PORT_COUNT                              4
#define VOLTAGE_MONITOR_VOLTAGE_MEASUREMENT_HISTORY_SIZE        8

#define VOLTAGE_MONITOR_WRITE_BUFFER_SIZE                       40

#define VOLTAGE_MONITOR_SCAN_RANGE_BEGIN_ADDRESS                0x48                            /// First address
#define VOLTAGE_MONITOR_SCAN_RANGE_END_ADDRESS                  0x4B                            /// Last address

#define VOLTAGE_MONITOR_FACTOR_A                0.9
#define VOLTAGE_MONITOR_FACTOR_B                0.1

#define VOLTAGE_MONITOR_SCAN_TIMEOUT 250

typedef struct port_voltage_measurement_history
{
    uint16_t mesurements[VOLTAGE_MONITOR_VOLTAGE_MEASUREMENT_HISTORY_SIZE];
    uint16_t rms_voltage_measurement;
    uint16_t current_write_index;
    uint16_t current_read_index;
    bool is_port_active;
} VM_port_voltage_measurement_history_t;

typedef struct channel_voltage_monitor
{
    VM_port_voltage_measurement_history_t ports_mesurement_objs[VOLTAGE_MONITOR_PORT_COUNT];
    bool channel_enabled;
    bool channel_active;                /// channel currently used for communication
} VM_channel_voltage_monitor_t;

typedef struct voltage_monitor
{
    VM_channel_voltage_monitor_t channels[VOLTAGE_MONITOR_CHANNEL_COUNT];

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    adc_oneshot_unit_handle_t adc1_handle;
} VM_voltage_monitor_t;


/// @brief Initialize voltage monitor
void VM_Init();

/// @brief 
void VM_Update();

/// @brief 
/// @param channel_index 
/// @param port_to_measure 
/// @return 
bool VM_PrepareToMeasure(uint8_t channel_index, uint8_t port_to_measure);

/// @brief 
/// @param channel_index 
/// @param port_to_measure 
/// @return 
bool VM_MeasurePortFromChannel(uint8_t channel_index,uint8_t port_to_measure);

/// @brief 
void VM_LogDataToSerialMonitor();

/// @brief 
void VM_CalculateWeightedMean();

/// @brief 
void VM_ScanForChannelsAndPorts();

/// @brief 
/// @return 
VM_voltage_monitor_t* VM_GetVoltageMonitor();

#endif