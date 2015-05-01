#ifndef QUATERNION_H_
#define QUATERNION_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

float QuaternionNorm(float quat[4]);

// -----------------------------------------------------------------------------
void QuaternionNormalize(float quat[4]);

// -----------------------------------------------------------------------------
void QuaternionFromVectors(float v1[3], float v2[3], float quat[4]);


#endif  // QUATERNION_H_
