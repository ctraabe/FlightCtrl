#include "quaternion.h"

#include <math.h>


// =============================================================================
// Private data:

#define QUAT_NORMALIZATION_GAIN (0.5)


// =============================================================================
// Public functions:

float * QuaternionMultiply(float quat1[4], float quat2[4], float result[4])
{
  result[0] = quat1[0] * quat2[0] - quat1[1] * quat2[1] - quat1[2] * quat2[2]
    - quat1[3] * quat2[3];
  result[1] = quat1[0] * quat2[1] + quat1[1] * quat2[0] + quat1[2] * quat2[3]
    - quat1[3] * quat2[2];
  result[2] = quat1[0] * quat2[2] - quat1[1] * quat2[3] + quat1[2] * quat2[0]
    + quat1[3] * quat2[1];
  result[3] = quat1[0] * quat2[3] + quat1[1] * quat2[2] - quat1[2] * quat2[1]
    + quat1[3] * quat2[0];

  return result;
}

// -----------------------------------------------------------------------------
float QuaternionNorm(float quat[4])
{
  return sqrt(square(quat[0]) + square(quat[1]) + square(quat[2])
    + square(quat[3]));
}

// -----------------------------------------------------------------------------
float * QuaternionNormalize(float quat[4])
{
  // The following code pushes the quaternion toward unity and is much more
  // efficient than normalization (no sqrt and no divide).
  float norm_correction = QUAT_NORMALIZATION_GAIN * (1.0 - quat[0] * quat[0]
    - quat[1] * quat[1] - quat[2] * quat[2] - quat[3] * quat[3]);

  quat[0] += quat[0] * norm_correction;
  quat[1] += quat[1] * norm_correction;
  quat[2] += quat[2] * norm_correction;
  quat[3] += quat[3] * norm_correction;

  return quat;
}

// -----------------------------------------------------------------------------
float * QuaternionRotateVector(float quat[4], float v[3], float result[3])
{
  float temp, r_2[3][3];

  r_2[0][0] = square(quat[0]);
  r_2[1][1] = r_2[0][0];
  r_2[2][2] = r_2[0][0];
  r_2[0][0] += square(quat[1]) - 0.5;
  r_2[1][1] += square(quat[2]) - 0.5;
  r_2[2][2] += square(quat[3]) - 0.5;

  r_2[1][0] = quat[0] * quat[3];
  r_2[0][1] = -r_2[1][0];
  temp = quat[1] * quat[2];
  r_2[1][0] += temp;
  r_2[0][1] += temp;

  r_2[0][2] = quat[0] * quat[2];
  r_2[2][0] = -r_2[0][2];
  temp = quat[1] * quat[3];
  r_2[0][2] += temp;
  r_2[2][0] += temp;

  r_2[2][1] = quat[0] * quat[1];
  r_2[1][2] = -r_2[2][1];
  temp = quat[2] * quat[3];
  r_2[2][1] += temp;
  r_2[1][2] += temp;

  result[0] = 2.0 * (r_2[0][0] * v[0] + r_2[0][1] * v[1] + r_2[0][2] * v[2]);
  result[1] = 2.0 * (r_2[1][0] * v[0] + r_2[1][1] * v[1] + r_2[1][2] * v[2]);
  result[2] = 2.0 * (r_2[2][0] * v[0] + r_2[2][1] * v[1] + r_2[2][2] * v[2]);

  return result;
}
