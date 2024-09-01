

#include "voltage_monitor.h"

VM_voltage_monitor_t VM_Obj;
char log_write_buffer[VOLTAGE_MONITOR_WRITE_BUFFER_SIZE];

adc_oneshot_unit_init_cfg_t init_config1 = 
{
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
};

i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,  // Set your desired clock speed
};

void VM_Init()
{
    memset(&VM_Obj,0,sizeof(VM_Obj));

    // ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &VM_Obj.bus_handle));
    // ESP_ERROR_CHECK(i2c_master_bus_add_device(VM_Obj.bus_handle, &dev_cfg, &VM_Obj.dev_handle));
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &VM_Obj.adc1_handle));

    for (uint8_t channel = 0; channel < 8;channel++)
    {
        VM_Obj.channels[channel].channel_enabled = (VOLTAGE_MONITOR_ENABLED_CHANEL_BITFIELD >> channel) & 0x01;
    }
}

void VM_Update()
{
    for(uint8_t channel = 0; channel < VOLTAGE_MONITOR_CHANNEL_COUNT;channel++)
    {
        for(uint8_t port=0; port < VOLTAGE_MONITOR_PORT_COUNT; port++)
        {
            VM_PrepareToMeasure(channel,port);
            VM_MeasurePortFromChannel(channel, port);
        }
    }
    VM_CalculateWeightedMean();

    VM_LogDataToSerialMonitor();
}

bool VM_PrepareToMeasure(uint8_t channel_index, uint8_t port_to_measure)
{
    if(VM_Obj.channels[channel_index].channel_enabled == 0)
        return false;
    // TODO Implement preparation when will used more than one channel
    return true;
}

bool VM_MeasurePortFromChannel(uint8_t channel_index, uint8_t port_to_measure)
{
    if(VM_Obj.channels[channel_index].channel_enabled == 0)
        return false;
#if VOLTAGE_MONITOR_USE_BUILDIN_ADC_ENABLE
    if(port_to_measure < 4)
    {
        int raw_data = 0;
        uint16_t* write_index;
        switch (port_to_measure)
        {
        case 0:
            write_index = &VM_Obj.channels[channel_index]
                                    .ports_mesurement_objs[port_to_measure]
                                    .current_write_index;

            ESP_ERROR_CHECK(adc_oneshot_read(VM_Obj.adc1_handle, ADC_CHANNEL_0, &raw_data));
            *write_index = (*write_index + 1) % VOLTAGE_MONITOR_VOLTAGE_MEASUREMENT_HISTORY_SIZE;
            VM_Obj.channels[channel_index]
                    .ports_mesurement_objs[port_to_measure]
                    .mesurements[*write_index] = (uint16_t)raw_data;
            break;
        case 1:
            write_index = &VM_Obj.channels[channel_index]
                                            .ports_mesurement_objs[port_to_measure]
                                            .current_write_index;

            ESP_ERROR_CHECK(adc_oneshot_read(VM_Obj.adc1_handle, ADC_CHANNEL_1, &raw_data));
            *write_index = (*write_index + 1) % VOLTAGE_MONITOR_VOLTAGE_MEASUREMENT_HISTORY_SIZE;
            VM_Obj.channels[channel_index]
                    .ports_mesurement_objs[port_to_measure]
                    .mesurements[*write_index] = (uint16_t)raw_data;
            break;
        case 2:
            write_index = &VM_Obj.channels[channel_index]
                                    .ports_mesurement_objs[port_to_measure]
                                    .current_write_index;

            ESP_ERROR_CHECK(adc_oneshot_read(VM_Obj.adc1_handle, ADC_CHANNEL_2, &raw_data));
            *write_index = (*write_index + 1) % VOLTAGE_MONITOR_VOLTAGE_MEASUREMENT_HISTORY_SIZE;
            VM_Obj.channels[channel_index]
                    .ports_mesurement_objs[port_to_measure]
                    .mesurements[*write_index] = (uint16_t)raw_data;
            break;
        case 3:
            write_index = &VM_Obj.channels[channel_index]
                                    .ports_mesurement_objs[port_to_measure]
                                    .current_write_index;

            ESP_ERROR_CHECK(adc_oneshot_read(VM_Obj.adc1_handle, ADC_CHANNEL_3, &raw_data));
            *write_index = (*write_index + 1) % VOLTAGE_MONITOR_VOLTAGE_MEASUREMENT_HISTORY_SIZE;
            VM_Obj.channels[channel_index]
                    .ports_mesurement_objs[port_to_measure]
                    .mesurements[*write_index] = (uint16_t)raw_data;
            break;
        default:
            break;
        }
    }
#else
// TODO Implement request to adc via I2C
#endif
    return true;
}

void VM_LogDataToSerialMonitor()
{
    for(uint8_t channel = 0; channel < VOLTAGE_MONITOR_CHANNEL_COUNT;channel++)
    {
        if(VM_Obj.channels[channel].channel_enabled == 0)
            esp_log_write(ESP_LOG_INFO, "*", "*** CHANNEL %1d DISABLED ***\n", channel);

        for(uint8_t port = 0; port < VOLTAGE_MONITOR_PORT_COUNT; port++)
        {
            if(VM_Obj.channels[channel].channel_enabled == 0)
                continue;

            uint16_t steps = (1 << ADC_BITNESS_USED) - 1;//;4047;
            uint16_t ref_voltage = REF_VOLTAGE_3300_MV;
            uint16_t weighted_mean = VM_Obj.channels[channel]
                                            .ports_mesurement_objs[port]
                                            .rms_voltage_measurement;

            uint16_t voltage = (uint16_t)(weighted_mean/(float)steps * ref_voltage);

            char channel_buff[2];
            char port_buff[3];
            char measure_buffer[8];
            char voltage_buffer[8];

            itoa(channel        , channel_buff  , 10);
            itoa(port           , port_buff     , 10);
            itoa(weighted_mean  , measure_buffer, 10);
            itoa(voltage        , voltage_buffer, 10);

            memset(&log_write_buffer, 0, VOLTAGE_MONITOR_WRITE_BUFFER_SIZE);
            snprintf(log_write_buffer, VOLTAGE_MONITOR_WRITE_BUFFER_SIZE,
                        "[ C%1s P%2s ]:  %8s - %8s mV\n",
                        channel_buff, port_buff, measure_buffer, voltage_buffer);

            esp_log_write(ESP_LOG_INFO, "*", log_write_buffer);
        }
    }
}

void VM_CalculateWeightedMean()
{
    for(uint8_t channel = 0; channel < VOLTAGE_MONITOR_CHANNEL_COUNT;channel++)
    {
        for(uint8_t port=0; port < VOLTAGE_MONITOR_PORT_COUNT; port++)
        {
            while( VM_Obj.channels[channel].ports_mesurement_objs[port].current_read_index !=
                VM_Obj.channels[channel].ports_mesurement_objs[port].current_write_index)
            {
                uint16_t* read_index = &VM_Obj.channels[channel]
                                .ports_mesurement_objs[port]
                                .current_read_index;
                
                *read_index = (*read_index + 1) % VOLTAGE_MONITOR_VOLTAGE_MEASUREMENT_HISTORY_SIZE;

                VM_Obj.channels[channel]
                .ports_mesurement_objs[port].rms_voltage_measurement =
                    VM_Obj.channels[channel]                    /// Previously calculated weighted value
                    .ports_mesurement_objs[port].rms_voltage_measurement * VOLTAGE_MONITOR_FACTOR_A 
                + VM_Obj.channels[channel]                      /// New value
                .ports_mesurement_objs[port]
                .mesurements[*read_index] * VOLTAGE_MONITOR_FACTOR_B;
            }
        }
    } 
}

void VM_ScanForChannelsAndPorts()
{
    for (uint8_t channel = 0; channel < VOLTAGE_MONITOR_CHANNEL_COUNT; channel++)
    {
        printf("*** SCAN FOR PORT FOR CHANNEL %d ***\n", channel);
        if (VM_Obj.channels[channel].channel_enabled == 0)
        {
            printf("***      CHANNEL %d DISABLED     ***\n", channel);
            continue;
        }
        for (uint8_t i = VOLTAGE_MONITOR_SCAN_RANGE_BEGIN_ADDRESS; 
                     i <= VOLTAGE_MONITOR_SCAN_RANGE_END_ADDRESS; i++)
        {
            int ret;
            uint8_t address = (i << 1) | I2C_MASTER_WRITE;  // Shift address and add write bit
            uint8_t dummy_data = 0;  // Dummy data to send
            
            // Using new I2C write function
            ret = i2c_master_write_to_device(I2C_PORT, address, &dummy_data, 0, VOLTAGE_MONITOR_SCAN_TIMEOUT / portTICK_PERIOD_MS);
            
            if (ret == ESP_OK)
            {
                printf("*** FOUND PORT %d: ADDRESS 0x%2x ***\n", (i - VOLTAGE_MONITOR_SCAN_RANGE_BEGIN_ADDRESS), i);
            }
        }
    }
}

VM_voltage_monitor_t *VM_GetVoltageMonitor()
{
    return &VM_Obj;
}
