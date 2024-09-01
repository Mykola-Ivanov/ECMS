

#ifndef MAIN_CFG_H
#define MAIN_CFG_H

#define VOLTAGE_MONITOR_ADC_12BIT               12
#define VOLTAGE_MONITOR_ADC_16BIT               16

#define ADC_BITNESS_USED                        VOLTAGE_MONITOR_ADC_12BIT

#define REF_VOLTAGE_5000_MV                     5000
#define REF_VOLTAGE_3300_MV                     3300

#define VOLTAGE_MONITOR_CHANNEL_COUNT           8

#define ADC_REF_VOLTAGE_IN_MV                   REF_VOLTAGE_3300_MV

#define I2C_PORT                                I2C_NUM_0
#define I2C_MASTER_SCL_IO                       22                              /// GPIO 22
#define I2C_MASTER_SDA_IO                       21                              /// GPIO 21

#define VOLTAGE_MONITOR_MEASURE_PERIOD_MS       12000

#define VOLTAGE_MONITOR_USE_BUILDIN_ADC_ENABLE  1

#define I2C_MASTER_FREQ_HZ                      100000                          /// 100 kHz I2C Frequency
#define I2C_MASTER_ADDRESS                      0x58
#define VOLTAGE_MONITOR_ENABLED_CHANEL_BITFIELD 0x03

#endif
