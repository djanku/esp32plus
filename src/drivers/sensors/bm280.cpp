
#include "esp32plus/drivers/sensors/bm280.h"


bool BMx280::begin_i2c(I2CMaster *master) {
    i2c = master;
    i2c_address = BM280_ADDR0;
}

bool BMx280::begin_i2c(I2CMaster *master, uint8_t address) {
    i2c = master;
    i2c_address = address;
}
