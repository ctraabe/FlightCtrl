#ifndef VECTOR_H_
#define VECTOR_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

float * VectorAdd(float v1[3], float v2[3], float result[3]);

// -----------------------------------------------------------------------------
float * VectorCopy(float source[3], float destination[3]);

// -----------------------------------------------------------------------------
float * VectorCross(float v1[3], float v2[3], float result[3]);

// -----------------------------------------------------------------------------
float VectorDot(float v1[3], float v2[3]);

// -----------------------------------------------------------------------------
float * VectorGain(float v[3], float gain, float result[3]);

// -----------------------------------------------------------------------------
float * VectorGainAndAccumulate(float v[3], float gain, float result[3]);

// -----------------------------------------------------------------------------
float VectorNorm(float v[3]);

// -----------------------------------------------------------------------------
float VectorNorm2(float v[3]);


#endif  // VECTOR_H_
