

#include "esp_log.h"

#include "esp32plus/common.h"
#include "esp32plus/semaphore.h"
#include "esp32plus/i2c.h"
#include "esp32plus/utils.h"


#define ADDR_READ(addr) (addr << 1 | I2C_MASTER_READ)
#define ADDR_WRITE(addr) (addr << 1 | I2C_MASTER_WRITE)


static const char TAG[] = "i2c";


I2CMasterCommand::I2CMasterCommand(I2CMaster &master) : master(master) {
    ESP_LOGD(TAG, "I2C CMD");
    master.mutex.take(0);
    cmd = i2c_cmd_link_create();
}

I2CMasterCommand::~I2CMasterCommand() {
    ESP_LOGD(TAG, "I2C ~CMD");
    if (err != ESP_OK) {
        i2c_reset_rx_fifo(master.i2c_num);
        i2c_reset_tx_fifo(master.i2c_num);
    }
    i2c_cmd_link_delete(cmd);
    master.mutex.give();
}

void I2CMasterCommand::reset() {
    i2c_cmd_link_delete(cmd);
    cmd = i2c_cmd_link_create();
    if (err != ESP_OK) {
        i2c_reset_rx_fifo(master.i2c_num);
        i2c_reset_tx_fifo(master.i2c_num);
    }
    err = ESP_OK;
}

inline esp_err_t I2CMasterCommand::start() {
    err |= i2c_master_start(cmd);
    return err;
}

inline esp_err_t I2CMasterCommand::stop() {
    err |= i2c_master_stop(cmd);
    return err;
}

esp_err_t I2CMasterCommand::begin(TickType_t ticks_to_wait) {
    err |= i2c_master_cmd_begin(master.i2c_num, cmd, ticks_to_wait);
    return err;
}

esp_err_t I2CMasterCommand::write(uint8_t data, bool ack) {
    err |= i2c_master_write_byte(cmd, data, ack);
    return err;
}

esp_err_t I2CMasterCommand::write(uint8_t *data, size_t data_len, bool ack) {
    if (data_len < 1) 
        return ESP_ERR_INVALID_ARG;
    if (data_len == 1) 
        return write(data[0], ack);
    
    err |= i2c_master_write(cmd, data, data_len, ack);
    return err;
}

esp_err_t I2CMasterCommand::read(uint8_t *buffer, size_t count, i2c_ack_type_t ack) {
    if (!count)
        return ESP_ERR_INVALID_ARG;
    if (count == 1) {
        err |= i2c_master_read_byte(cmd, buffer, ack);
        return err;
    }
    err |= i2c_master_read(cmd, buffer, count, ack);
    return err;
}


esp_err_t I2CMasterCommand::status() const {
    return err;
}


I2CMaster::I2CMaster() : read_timeout(milis(500)) {
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
}


esp_err_t I2CMaster::setup(gpio_num_t scl, gpio_num_t sda, i2c_port_t num, 
                           int freq) {
    i2c_num = num;
    i2c_config.master.clk_speed = freq;
    i2c_config.scl_io_num = scl;
    i2c_config.sda_io_num = sda;

    esp_err_t ret = i2c_param_config(num, &i2c_config);
    if (ret != ESP_OK) 
        return ret;
    return i2c_driver_install(num, I2C_MODE_MASTER, 0, 0, ESP_INTR_FLAG_LOWMED);
}


esp_err_t I2CMaster::shutdown() {
    return i2c_driver_delete(i2c_num);
}


esp_err_t I2CMaster::read_byte(uint8_t slave_addr, uint8_t slave_reg, uint8_t *buffer) {
    return read_bytes(slave_addr, slave_reg, buffer, 1);
}

esp_err_t I2CMaster::read_bytes(uint8_t slave_addr, uint8_t slave_reg, 
                          uint8_t *buffer, size_t data_len) {
    I2CMasterCommand cmd(*this);
    cmd.start();
    cmd.write(ADDR_WRITE(slave_addr));
    cmd.write(slave_reg);
    cmd.stop();
    cmd.begin(read_timeout);
    cmd.reset();

    cmd.start();
    cmd.write(ADDR_READ(slave_addr));
    cmd.read(buffer, data_len, I2C_MASTER_LAST_NACK);
    cmd.stop();

    return cmd.begin(read_timeout);
}


esp_err_t I2CMaster::read_bit(uint8_t slave_addr, uint8_t slave_reg, uint8_t pos, bool *bit) {
    uint8_t data;
    read_byte(slave_addr, slave_reg, &data);
    *bit = get_bit(data, pos) ? true : false;
    return ESP_OK;
}

esp_err_t I2CMaster::read_bits(uint8_t slave_addr, uint8_t slave_reg,
                               uint8_t pos_end, uint8_t pos_start, uint8_t *bits) {
    uint8_t data;
    read_byte(slave_addr, slave_reg, &data);
    *bits = get_bits(data, pos_end, pos_start);
    return ESP_OK;
}

esp_err_t I2CMaster::write_bytes(uint8_t slave_addr, uint8_t slave_reg, uint8_t *data,
                                 size_t data_len) {
    I2CMasterCommand cmd(*this);
    cmd.start();
    cmd.write(ADDR_WRITE(slave_addr));
    cmd.write(slave_reg);
    if (data_len > 1) 
        cmd.write(data, data_len-1, 0);
    cmd.write(data[data_len-1]);
    cmd.stop();
    return cmd.begin(milis(1000));
}


esp_err_t I2CMaster::write_byte(uint8_t slave_addr, uint8_t slave_reg, uint8_t data) {
    return write_bytes(slave_addr, slave_reg, &data, 1);
}


esp_err_t I2CMaster::write_bit(uint8_t slave_addr, uint8_t slave_reg, uint8_t pos, 
                               bool value) {
    uint8_t b;
    read_byte(slave_addr, slave_reg, &b);
    b = set_bit(b, pos, value);
    return write_byte(slave_addr, slave_reg, b);
}


esp_err_t I2CMaster::write_bits(uint8_t slave_addr, uint8_t slave_reg,
                                uint8_t pos_end, uint8_t pos_start, uint8_t value) {
    uint8_t tmp;
    read_byte(slave_addr, slave_reg, &tmp);
    tmp = set_bits(tmp, pos_end, pos_start, value);
    return write_byte(slave_addr, slave_reg, tmp);
}


bool I2CMaster::is_slave_present(uint8_t slave_addr) {
    I2CMasterCommand cmd(*this);

    cmd.start();
    cmd.write(ADDR_WRITE(slave_addr));
    cmd.stop();

    if (cmd.status() != ESP_OK)
        return false;

    return cmd.begin(milis(25));
}
