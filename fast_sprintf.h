#ifndef FAST_SPRINTF_H
#define FAST_SPRINTF_H


#include <inttypes.h>


// =============================================================================
// Public functions:

uint8_t SPrintfEOL(uint8_t *char_array);
uint8_t SPrintfSpace(uint8_t *char_array);

uint8_t SPrintfU8(uint8_t *char_array, uint16_t value);
uint8_t SPrintfU16(uint8_t *char_array, uint16_t value);

uint8_t SPrintfS8(uint8_t *char_array, int8_t value);
uint8_t SPrintfS16(uint8_t *char_array, int16_t value);


#endif  // FAST_SPRINTF_H
