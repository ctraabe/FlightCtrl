#include "attitude.h"

#include <math.h>

#include "adc.h"
#include "quaternion.h"


// =============================================================================
// Private data:

static float quat_[4], g_b_[3];


// =============================================================================
// Public functions:

void UpdateQuaternion(void)
{
  #define DT (1 / 128.0)

  float p_2 = AngularRate(X_AXIS) * 0.5 * DT;
  float q_2 = AngularRate(Y_AXIS) * 0.5 * DT;
  float r_2 = AngularRate(Z_AXIS) * 0.5 * DT;

  float d_quat[4];
  d_quat[0] = -p_2 * quat_[1] - q_2 * quat_[2] - r_2 * quat_[3];
  d_quat[1] =  p_2 * quat_[0] - q_2 * quat_[3] + r_2 * quat_[2];
  d_quat[2] =  p_2 * quat_[3] + q_2 * quat_[0] - r_2 * quat_[1];
  d_quat[3] = -p_2 * quat_[2] + q_2 * quat_[1] + r_2 * quat_[0];

  quat_[0] += d_quat[0];
  quat_[1] += d_quat[1];
  quat_[2] += d_quat[2];
  quat_[3] += d_quat[3];

  QuaternionNormalize(quat_);
}

// -----------------------------------------------------------------------------
float HeadingAngle(void)
{
  return atan2(2.0 * quat_[0] * quat_[3] + quat_[1] * quat_[2], 1.0 - 2.0
    * (quat_[2] * quat_[2] + quat_[3] * quat_[3]));
}

// -----------------------------------------------------------------------------
void UpdateGravtiyInBody(void)
{
  g_b_[0] = 2.0 * (quat_[1] * quat_[3] - quat_[0] * quat_[2]);
  g_b_[1] = 2.0 * (quat_[2] * quat_[3] - quat_[0] * quat_[1]);
  g_b_[2] = 2.0 * (quat_[0] * quat_[0] + quat_[3] * quat_[3]) - 1.0;
}

// -----------------------------------------------------------------------------
void CompareAttitudeAccelerometer(void)
{
  float quat_acceleration_2_g_b[4];
  QuaternionFromVectors(AccelerationVector(), g_b_, quat_acceleration_2_g_b);
}
