
#pragma GCC diagnostic ignored "-Wshift-negative-value"


#include "esp32plus/utils.h"


uint8_t set_bit(uint8_t data, uint8_t pos, uint8_t value) {
    return value ? (data | (1 << pos)) : data & ((~value) ^ (1 << pos));
}


uint8_t set_bits(uint8_t data, uint8_t pos_end, uint8_t pos_start, uint8_t value) {
    if (pos_start > pos_end) 
        return 0;
    uint8_t mask = ~((~(uint8_t)0) << (pos_end-pos_start+1));
    value = value & mask;
    mask = ~(mask << pos_start);
    return ((data & mask) | (value << pos_start));
}


bool get_bit(uint8_t data, uint8_t pos) {
    return (data & (1 << pos)) ? true : false;
}


uint8_t get_bits(uint8_t data, uint8_t pos_end, uint8_t pos_start) {
    if (pos_start > pos_end)
        return 0;
    uint8_t mask = (~((~(uint8_t)0) << (pos_end-pos_start+1))) << pos_start;
    return (data & mask) >> pos_start;
}
