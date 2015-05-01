#include "quaternion.h"

#include <math.h>

#include "vector.h"


// =============================================================================
// Public functions:

float QuaternionNorm(float quat[4])
{
  return sqrt(square(quat[0]) + square(quat[1]) + square(quat[2])
    + square(quat[3]));
}

// -----------------------------------------------------------------------------
void QuaternionNormalize(float quat[4])
{
  float temp = 1.0 / QuaternionNorm(quat);
  quat[0] *= temp;
  quat[1] *= temp;
  quat[2] *= temp;
  quat[3] *= temp;
}

// -----------------------------------------------------------------------------
void QuaternionFromVectors(float v1[3], float v2[3], float quat[4])
{
  quat[0] = VectorDot(v1, v2);
  VectorCross(v1, v2, &quat[1]);
  float temp = square(quat[1]) + square(quat[2]) + square(quat[3]);
  quat[0] += sqrt(square(quat[0]) + temp);
  temp = 1.0 / sqrt(square(quat[0]) + temp);
  quat[0] *= temp;
  quat[1] *= temp;
  quat[2] *= temp;
  quat[3] *= temp;
}
