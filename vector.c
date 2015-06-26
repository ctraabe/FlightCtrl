#include "vector.h"

#include <math.h>


// =============================================================================
// Public functions:

void VectorCross(float v1[3], float v2[3], float result[3])
{
  result[0] = v1[1] * v2[2] - v1[2] * v2[1];
  result[1] = v1[2] * v2[0] - v1[0] * v2[2];
  result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// -----------------------------------------------------------------------------
float VectorDot(float v1[3], float v2[3])
{
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// -----------------------------------------------------------------------------
float VectorNorm(float v[3])
{
  return sqrt(square(v[1]) + square(v[2]) + square(v[3]));
}
