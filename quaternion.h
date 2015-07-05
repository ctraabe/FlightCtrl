#ifndef QUATERNION_H_
#define QUATERNION_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

float * QuaternionInverse(float quat[4], float result[4]);

// -----------------------------------------------------------------------------
float * QuaternionMultiply(float quat1[4], float quat2[4], float result[4]);

// -----------------------------------------------------------------------------
float QuaternionNorm(float quat[4]);

// -----------------------------------------------------------------------------
float * QuaternionNormalize(float quat[4]);

// -----------------------------------------------------------------------------
float * QuaternionRotateVector(float quat[4], float v[3], float result[3]);


#endif  // QUATERNION_H_
