

#pragma once


#include "esp_err.h"
#include "esp_pthread.h"
#include "pthread.h"
#include "driver/i2c.h"

#include "tools.h"


#define SDA GPIO_NUM_21
#define SCL GPIO_NUM_22

#define ADDR_READ(addr) (addr << 1 | I2C_MASTER_READ)
#define ADDR_WRITE(addr) (addr << 1 | I2C_MASTER_WRITE)


class I2CMaster;

// TODO: nemusi se volat explicitne start, da se to zaridit

class I2CMasterCommand {
    public:
        I2CMasterCommand(I2CMaster&);
        ~I2CMasterCommand();

        void reset();

        inline esp_err_t start();
        inline esp_err_t stop();
        esp_err_t write(uint8_t data, bool ack = true);
        esp_err_t write(uint8_t *data, size_t data_len, bool ack = true);
        esp_err_t read(uint8_t *buffer, size_t count, i2c_ack_type_t ack = I2C_MASTER_ACK);

        esp_err_t begin(TickType_t ticks_to_wait = 0);

        esp_err_t status() const;
    private:
        esp_err_t err = ESP_OK;
        i2c_cmd_handle_t cmd;
        I2CMaster &master;
};


class I2CMaster {
    public:
        I2CMaster();

        esp_err_t setup(gpio_num_t scl = SCL, gpio_num_t sda = SDA, 
                        i2c_port_t num = I2C_NUM_0, int freq = 100000);
        esp_err_t finish();


        esp_err_t read_byte(uint8_t slave_addr, uint8_t slave_reg, uint8_t *buffer);
        esp_err_t read_bytes(uint8_t slave_addr, uint8_t slave_reg, uint8_t *buffer, 
                             size_t data_len);

        esp_err_t write_byte(uint8_t slave_addr, uint8_t slave_reg, uint8_t data);
        esp_err_t write_bytes(uint8_t slave_addr, uint8_t slave_reg, uint8_t *data, 
                              size_t data_len);
        esp_err_t write_bit(uint8_t slave_addr, uint8_t slave_reg, uint8_t pos,
                            uint8_t value);

        // esp_err_t write_bit()

        bool is_slave_present(uint8_t slave_addr);

        TickType_t read_timeout;
    protected:
        i2c_config_t i2c_config;
        i2c_port_t i2c_num; 
        bool setuped = false;
    private:
        esp_err_t _select_register(uint8_t slave_addr, uint8_t slave_reg);

        Mutex mutex;
        friend class I2CMasterCommand;
};


