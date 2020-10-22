
#pragma once

#include "esp32plus/i2c.h"

#define BMX280_ADDR0 0x76
#define BMX280_ADDR1 0x77

#define BMX280_REG_DIG_T1_LSB 0x88
#define BMX280_REG_DIG_T2_LSB 0x8A
#define BMX280_REG_DIG_T3_LSB 0x8C
#define BMX280_REG_ID 0xD0
#define BMX280_REG_RESET 0xE0
#define BMX280_REG_STATUS 0xF3
#define BMX280_REG_CTRL_MEAS 0xF4
#define BMX280_REG_CONFIG 0xF5
#define BMX280_REG_PRESSURE_MSB 0xF7
#define BMX280_REG_TEMPERATURE_MSB 0xFA

#define BMP280_ID 0x58
#define BME280_ID 0x60


class BMP280 : public I2CMasterDriver {
    public:
        enum Mode {
            SLEEP_MODE = 0x00,
            FORCED_MODE = 0x01,
            NORMAL_MODE = 0x03
        };

        enum Sampling {
            SAMPLING_X0 = 0x00,
            SAMPLING_X1 = 0x01,
            SAMPLING_X2 = 0x02,
            SAMPLING_X4 = 0x03,
            SAMPLING_X8 = 0x04,
            SAMPLING_X16 = 0x05,
        };

        enum StandbyMS {
            STANDBY_1MS = 0x00,
            STANDBY_63MS = 0x01,
            STANDBY_125MS = 0x02,
            STANDBY_250MS = 0x03,
            STANDBY_500MS = 0x04,
            STANDBY_100MS = 0x05,
            STANDBY_200MS = 0x06,
            STANDBY_400MS = 0x07,
        };

        enum Filter {
            FILTER_OFF = 0x00,
            FILTER_X2 = 0x01,
            FILTER_X4 = 0x02,
            FILTER_X8 = 0x03,
            FILTER_X16 = 0x04
        };

        BMP280() : bmx_chip_id(BMP280_ID) {};

        esp_err_t begin_i2c(I2CMaster *master, uint8_t address = BMX280_ADDR0);
        
        esp_err_t set_mode(Mode mode);
        esp_err_t set_standby(StandbyMS standby);
        esp_err_t set_filter(Filter filter);
        esp_err_t set_sampling(Sampling temperature, Sampling pressure);

        esp_err_t load_compensation_data();

        esp_err_t read_temperature(float &temp);
		esp_err_t read_pressure(float &pressure, bool read_temp = false);
		esp_err_t read_chip_id(uint8_t &chip_id);
    
        esp_err_t reset();

        float compute_altitude(float pressure, float temperature);

    protected:
		struct BMX280Calibration {
            uint16_t dig_t1;
            int16_t dig_t2;
            int16_t dig_t3;
            uint16_t dig_p1;
            int16_t dig_p2;
            int16_t dig_p3;
            int16_t dig_p4;
            int16_t dig_p5;
            int16_t dig_p6;
            int16_t dig_p7;
            int16_t dig_p8;
            int16_t dig_p9;
        } calib;
        uint8_t bmx_chip_id;
        int64_t temp_fine = 0;
};


class BME280: public BMP280 {
    public:
        BME280() { bmx_chip_id = BME280_ID; };
    protected:
};


