
#include <math.h>

#include "esp_log.h"

#include "esp32plus/drivers/sensors/bmx280.h"


#define SEALEVEL_PRESSURE 101325.0
#define ALT_PRESSURE_POW 0.1902225603956629

static const char *TAG = "BMX280";


esp_err_t BMP280::begin_i2c(I2CMaster *master, uint8_t address) {
    i2c = master;
    i2c_address = address;

    esp_err_t err;

    uint8_t chip_id;
    err = read_chip_id(chip_id);
    if (err != ESP_OK) return err;
    if (chip_id != bmx_chip_id) {
        ESP_LOGW(TAG, "Chip ID != 0x%02x, it's 0x%02x", bmx_chip_id, chip_id);
    }

    err = load_compensation_data();
    if (err != ESP_OK) 
        return err;

    // default setup 
    err = set_mode(NORMAL_MODE) | 
          set_standby(STANDBY_1MS) | 
          set_filter(FILTER_OFF) | 
          set_sampling(SAMPLING_X1, SAMPLING_X1);
    return err;
}

esp_err_t BMP280::set_mode(Mode mode) {
    return i2c->write_bits(i2c_address, BMX280_REG_CTRL_MEAS, 1, 0, mode);
}

esp_err_t BMP280::set_standby(StandbyMS standby) {
    return i2c->write_bits(i2c_address, BMX280_REG_CONFIG, 7, 5, standby);
}

esp_err_t BMP280::set_filter(Filter filter) {
    return i2c->write_bits(i2c_address, BMX280_REG_CONFIG, 4, 2, filter);
}

esp_err_t BMP280::set_sampling(Sampling temp_s, Sampling press_s) { 
    return i2c->write_bits(i2c_address, BMX280_REG_CTRL_MEAS, 7, 5, temp_s) |
           i2c->write_bits(i2c_address, BMX280_REG_CTRL_MEAS, 4, 2, press_s);
}

esp_err_t BMP280::reset() {
    return i2c->write_byte(i2c_address, BMX280_REG_RESET, 0xB6);
}

esp_err_t BMP280::read_temperature(float &temp) {
    // read temperature data 
    uint8_t err;
    uint8_t buffer[3];
    err = i2c->read_bytes(i2c_address, BMX280_REG_TEMPERATURE_MSB, buffer, 3);
    if (err != ESP_OK)
        return err;
    
    int32_t adc_t = ((uint32_t) buffer[0] << 12) | 
                    ((uint32_t) buffer[1] << 4) | 
                    ((buffer[2] >> 4) & 0x0f);
	int64_t var1, var2;
	var1 = (((adc_t>>3) - ((int32_t) calib.dig_t1<<1)) * 
            ((int32_t) calib.dig_t2)) >> 11;
	var2 = (((((adc_t>>4) - ((int32_t) calib.dig_t1)) * 
            ((adc_t>>4) - ((int32_t) calib.dig_t1))) >> 12) *
	        ((int32_t) calib.dig_t3)) >> 14;
    temp_fine = var1 + var2;
    temp = ((temp_fine * 5 + 128) >> 8) / 100.0;
    return ESP_OK;
}

esp_err_t BMP280::read_pressure(float &pressure, bool read_temp) {
    uint8_t err;
    // we read temperature due to temp_fine variable used by pressure calc
    if (read_temp) {
        float temp;
        err = read_temperature(temp);
        if (err != ESP_OK)
            return err;
    }

    uint8_t buffer[3];
    err = i2c->read_bytes(i2c_address, BMX280_REG_PRESSURE_MSB, buffer, 3);
    if (err != ESP_OK)
        return err;

    int32_t adc_p = ((uint32_t) buffer[0] << 12) | 
                    ((uint32_t) buffer[1] << 4) | 
                    ((buffer[2] >> 4) & 0x0F);
    int64_t var1, var2, p_acc;
    var1 = temp_fine - 128000;
    var2 = var1 * var1 * (int64_t) calib.dig_p6;
    var2 = var2 + ((var1 * (int64_t) calib.dig_p5) << 17);
    var2 = var2 + (((int64_t) calib.dig_p4) << 35);
    var1 = ((var1 * var1 * (int64_t) calib.dig_p3) >> 8) + 
           ((var1 * (int64_t) calib.dig_p2) << 12); 
    var1 = (((((int64_t) 1) << 47) + var1))*((int64_t) calib.dig_p1) >> 33;
    if (var1 == 0) {
        pressure = 0.0;
        return ESP_ERR_INVALID_ARG;
    }
  	p_acc = 1048576 - adc_p;
	p_acc = (((p_acc << 31) - var2) * 3125) / var1;
	var1 = (((int64_t) calib.dig_p9) * (p_acc >> 13) * (p_acc >> 13)) >> 25;
	var2 = (((int64_t) calib.dig_p8) * p_acc) >> 19;
	p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t) calib.dig_p7) << 4);
    pressure = p_acc / 256.0;
    return ESP_OK;
}

esp_err_t BMP280::read_chip_id(uint8_t &chip_id) {
    return i2c->read_byte(i2c_address, BMX280_REG_ID, &chip_id);
}

esp_err_t BMP280::load_compensation_data() {
    uint8_t buffer[24];
    esp_err_t err = i2c->read_bytes(i2c_address, BMX280_REG_DIG_T1_LSB, 
                                    buffer, 24);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to read compensation data");
        return err;
    }
    calib.dig_t1 = (uint16_t) ((buffer[1] << 8) + buffer[0]);
    calib.dig_t2 = (int16_t) ((buffer[3] << 8) + buffer[2]);
    calib.dig_t3 = (int16_t) ((buffer[5] << 8) + buffer[4]);
    calib.dig_p1 = (uint16_t) ((buffer[7] << 8) + buffer[7]);
    calib.dig_p2 = (int16_t) ((buffer[9] << 8) + buffer[9]);
    calib.dig_p3 = (int16_t) ((buffer[11] << 8) + buffer[10]);
    calib.dig_p4 = (int16_t) ((buffer[13] << 8) + buffer[12]);
    calib.dig_p5 = (int16_t) ((buffer[15] << 8) + buffer[14]);
    calib.dig_p6 = (int16_t) ((buffer[17] << 8) + buffer[16]);
    calib.dig_p7 = (int16_t) ((buffer[19] << 8) + buffer[18]);
    calib.dig_p8 = (int16_t) ((buffer[21] << 8) + buffer[20]);
    calib.dig_p9 = (int16_t) ((buffer[23] << 8) + buffer[22]);
    return ESP_OK;
}

float BMP280::compute_altitude(float pressure, float temperature) {
    return ((powf((SEALEVEL_PRESSURE / pressure), ALT_PRESSURE_POW) - 1.0) 
            * (temperature + 273.15)) / 0.0065;
}

