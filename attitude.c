#include "attitude.h"

#include <math.h>

#include "adc.h"
#include "quaternion.h"


// =============================================================================
// Private data:

static float quat_[4] = { 1.0, 0.0, 0.0, 0.0 }, g_b_[3] = { 0.0, 0.0, 1.0 };


// =============================================================================
// Private function declarations:

static void UpdateGravtiyInBody(void);
static void UpdateQuaternion(void);


// =============================================================================
// Accessors:

float Gravity(enum BodyAxes axis)
{
  return g_b_[axis];
}

// -----------------------------------------------------------------------------
float Quat(uint8_t n)
{
  return quat_[n];
}


// =============================================================================
// Public functions:

void UpdateAttitude(void)
{
  UpdateQuaternion();
  QuaternionNormalize(quat_);
  UpdateGravtiyInBody();
}

// -----------------------------------------------------------------------------
float HeadingAngle(void)
{
  return atan2(2.0 * quat_[0] * quat_[3] + quat_[1] * quat_[2], 1.0 - 2.0
    * (quat_[2] * quat_[2] + quat_[3] * quat_[3]));
}


// =============================================================================
// Private functions:

static void UpdateGravtiyInBody(void)
{
  g_b_[0] = 2.0 * (quat_[1] * quat_[3] - quat_[0] * quat_[2]);
  g_b_[1] = 2.0 * (quat_[2] * quat_[3] + quat_[0] * quat_[1]);
  g_b_[2] = 2.0 * (quat_[0] * quat_[0] + quat_[3] * quat_[3]) - 1.0;
}

// -----------------------------------------------------------------------------
static void UpdateQuaternion(void)
{
  float p_2_dt = AngularRate(X_BODY_AXIS) * 0.5 * DT;
  float q_2_dt = AngularRate(Y_BODY_AXIS) * 0.5 * DT;
  float r_2_dt = AngularRate(Z_BODY_AXIS) * 0.5 * DT;

  float d_quat[4];
  d_quat[0] = -p_2_dt * quat_[1] - q_2_dt * quat_[2] - r_2_dt * quat_[3];
  d_quat[1] =  p_2_dt * quat_[0] - q_2_dt * quat_[3] + r_2_dt * quat_[2];
  d_quat[2] =  p_2_dt * quat_[3] + q_2_dt * quat_[0] - r_2_dt * quat_[1];
  d_quat[3] = -p_2_dt * quat_[2] + q_2_dt * quat_[1] + r_2_dt * quat_[0];

  quat_[0] += d_quat[0];
  quat_[1] += d_quat[1];
  quat_[2] += d_quat[2];
  quat_[3] += d_quat[3];
}
