#ifndef VECTOR_H_
#define VECTOR_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

float VectorDot(float v1[3], float v2[3]);

// -----------------------------------------------------------------------------
void VectorCross(float v1[3], float v2[3], float result[3]);


#endif  // VECTOR_H_
