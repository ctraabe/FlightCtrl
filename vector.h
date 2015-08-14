#ifndef VECTOR_H_
#define VECTOR_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

float * VectorAdd(const float v1[3], const float v2[3], float result[3]);

// -----------------------------------------------------------------------------
float * VectorCopy(const float source[3], float destination[3]);

// -----------------------------------------------------------------------------
float * VectorCross(const float v1[3], const float v2[3], float result[3]);

// -----------------------------------------------------------------------------
float VectorDot(const float v1[3], const float v2[3]);

// -----------------------------------------------------------------------------
float * VectorGain(const float v[3], float gain, float result[3]);

// -----------------------------------------------------------------------------
float * VectorGainAndAccumulate(const float v[3], float gain, float result[3]);

// -----------------------------------------------------------------------------
float VectorNorm(const float v[3]);

// -----------------------------------------------------------------------------
float VectorNorm2(const float v[3]);


#endif  // VECTOR_H_
