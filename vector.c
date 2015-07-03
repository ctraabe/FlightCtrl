#include "vector.h"

#include <math.h>


// =============================================================================
// Public functions:

float * VectorAdd(float v1[3], float v2[3], float result[3])
{
  result[0] = v1[0] + v2[0];
  result[1] = v1[1] + v2[1];
  result[2] = v1[2] + v2[2];

  return result;
}

// -----------------------------------------------------------------------------
float * VectorCopy(float source[3], float destination[3])
{
  destination[0] = source[0];
  destination[1] = source[1];
  destination[2] = source[2];

  return destination;
}

// -----------------------------------------------------------------------------
float * VectorCross(float v1[3], float v2[3], float result[3])
{
  result[0] = v1[1] * v2[2] - v1[2] * v2[1];
  result[1] = v1[2] * v2[0] - v1[0] * v2[2];
  result[2] = v1[0] * v2[1] - v1[1] * v2[0];

  return result;
}

// -----------------------------------------------------------------------------
float VectorDot(float v1[3], float v2[3])
{
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// -----------------------------------------------------------------------------
float * VectorGain(float v[3], float gain, float result[3])
{
  result[0] = v[0] * gain;
  result[1] = v[1] * gain;
  result[2] = v[2] * gain;

  return result;
}

// -----------------------------------------------------------------------------
float * VectorGainAndAccumulate(float v[3], float gain, float result[3])
{
  result[0] =+ v[0] * gain;
  result[1] =+ v[1] * gain;
  result[2] =+ v[2] * gain;

  return result;
}

// -----------------------------------------------------------------------------
float VectorNorm(float v[3])
{
  return sqrt(VectorNorm2(v));
}

// -----------------------------------------------------------------------------
float VectorNorm2(float v[3])
{
  return square(v[0]) + square(v[1]) + square(v[2]);
}
