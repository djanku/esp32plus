
#pragma once

#include <stdint.h>

#define milis(ticks) (ticks / portTICK_RATE_MS)

uint8_t set_bit(uint8_t data, uint8_t pos, bool value);
uint8_t set_bits(uint8_t data, uint8_t pos_end, uint8_t pos_start, uint8_t value);
bool get_bit(uint8_t data, uint8_t pos);
uint8_t get_bits(uint8_t data, uint8_t pos_end, uint8_t pos_start);

