#ifndef MYMATH_H_
#define MYMATH_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

int16_t FloatToS16(float input);

// -----------------------------------------------------------------------------
int8_t S8Limit(int8_t input, int8_t lower_limit, int8_t upper_limit);

// -----------------------------------------------------------------------------
int16_t S16Limit(int16_t input, int16_t lower_limit, int16_t upper_limit);

// -----------------------------------------------------------------------------
int32_t S32Limit(int32_t input, int32_t lower_limit, int32_t upper_limit);

// -----------------------------------------------------------------------------
uint8_t U8Limit(uint8_t input, uint8_t lower_limit, uint8_t upper_limit);

// -----------------------------------------------------------------------------
uint16_t U16Limit(uint16_t input, uint16_t lower_limit, uint16_t upper_limit);

// -----------------------------------------------------------------------------
uint32_t U32Limit(uint32_t input, uint32_t lower_limit, uint32_t upper_limit);

// -----------------------------------------------------------------------------
float FloatLimit(float input, float lower_limit, float upper_limit);

// -----------------------------------------------------------------------------
// This function returns the result of "input" divided by two to the power of
// "power" and rounds the result to the nearest integer. It is equivalent to:
//   (int16_t)( round( (float)input / (float)( 2 ^ power ) ) )
// but much more efficient. It is left to the programmer to ensure that:
//   1) the largest expected result will not overflow the output type
//   2) the largest expected "input + 2^(power-1)" will not overflow the input
int16_t S16RoundRShiftS16(int16_t input, uint8_t power);

// -----------------------------------------------------------------------------
// Same as above but outputting int8_t
int8_t S8RoundRShiftS16(int16_t input, uint8_t power);

// -----------------------------------------------------------------------------
// Same as above but for int32_t (also outputs int32_t)
int32_t S32RoundRShiftS32(int32_t input, uint8_t power);

// -----------------------------------------------------------------------------
// Same as above but outputting int16_t
int16_t S16RoundRShiftS32(int32_t input, uint8_t power);

// -----------------------------------------------------------------------------
// Same as above but for uint16_t (also outputs uint16_t)
uint16_t U16RoundRShiftU16(uint16_t input, uint8_t power);

// -----------------------------------------------------------------------------
// Same as above but outputting uint8_t
uint8_t U8RoundRShiftU16(uint16_t input, uint8_t power);

// -----------------------------------------------------------------------------
// Same as above but for uint32_t (also outputs uint32_t)
uint32_t U32RoundRShiftU32(uint32_t input, uint8_t power);

// -----------------------------------------------------------------------------
// Same as above but outputting uint16_t
uint16_t U16RoundRShiftU32(uint32_t input, uint8_t power);


#endif  // MYMATH_H_
