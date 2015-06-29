#ifndef VECTOR_H_
#define VECTOR_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

float * VectorCross(float v1[3], float v2[3], float result[3]);

// -----------------------------------------------------------------------------
float VectorDot(float v1[3], float v2[3]);

// -----------------------------------------------------------------------------
float VectorNorm(float v[3]);


#endif  // VECTOR_H_
