
#pragma once

#include "esp32plus/i2c.h"

#define BM280_ADDR0 0x76
#define BM280_ADDR1 0x77

#define BM280_REG_DIG_T1_LSB 0x88
#define BM280_REG_DIG_T2_LSB 0x8A
#define BM280_REG_DIG_T3_LSB 0x8C
#define BM280_REG_ID 0xD0
#define BM280_REG_RESET 0xE0
#define BM280_REG_STATUS 0xF3
#define BM280_REG_CTRL_MEAS 0xF4
#define BM280_REG_CONFIG 0xF5
#define BM280_REG_PRESSURE_MSB 0xF7
#define BM280_REG_TEMPERATURE_MSB 0xFA

enum BmType {
    BMP280 = 0x58,
    BME280 = 0x60
};


class BMx280 : public I2CMasterDriver {
    bool begin_i2c(I2CMaster *master);
    bool begin_i2c(I2CMaster *master, uint8_t address);
};
