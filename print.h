#ifndef PRINT_H
#define PRINT_H

#include <inttypes.h>

uint8_t PrintEOL(uint8_t *char_array);
uint8_t PrintSpace(uint8_t *char_array);

uint8_t PrintU8(uint16_t value, uint8_t *char_array);
uint8_t PrintU16(uint16_t value, uint8_t *char_array);

uint8_t PrintS8(int8_t value, uint8_t *char_array);
uint8_t PrintS16(int16_t value, uint8_t *char_array);

#endif  // PRINT_H
