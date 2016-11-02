#include "attitude.h"

#include <math.h>

#include "adc.h"
#include "main.h"
#include "nav_comms.h"
#include "quaternion.h"
#include "vector.h"


// =============================================================================
// Private data:

#define ACCELEROMETER_CORRECTION_GAIN (0.01)

static float quat_[4] = { 1.0, 0.0, 0.0, 0.0 }, g_b_[3] = { 0.0, 0.0, 1.0 };
static float heading_angle_ = 0.0;
static uint8_t reset_attitude_ = 0;


// =============================================================================
// Private function declarations:

static float * CorrectQuaternionWithAccelerometer(float quat[4]);
static void HandleAttitudeReset(void);


// =============================================================================
// Accessors:

const float * GravityInBodyVector(void)
{
  return g_b_;
}

// -----------------------------------------------------------------------------
float HeadingAngle(void)
{
  return heading_angle_;
}

// -----------------------------------------------------------------------------
const float * Quat(void)
{
  return quat_;
}


// =============================================================================
// Public functions:

void UpdateAttitude(void)
{
  if (!reset_attitude_)
  {
    UpdateQuaternion(quat_, AngularRateVector(), DT);
    UpdateGravtiyInBody(quat_, g_b_);
    CorrectQuaternionWithAccelerometer(quat_);
    QuaternionNormalizingFilter(quat_);
  }
  else
  {
    HandleAttitudeReset();
  }
  if (NavStatus() & NAV_STATUS_BIT_HEADING_DATA_OK) CorrectHeading();
  UpdateGravtiyInBody(quat_, g_b_);
  heading_angle_ = HeadingFromQuaternion(quat_);
}

// -----------------------------------------------------------------------------
void CorrectHeading(void)
{
  // Disregard bad data
  // TODO: set some error bit
  if (HeadingCorrection0() > 1.0 || HeadingCorrection0() < 0.95) return;
  if (fabs(HeadingCorrectionZ()) > 0.05) return;

  float quat_corection[4] = { HeadingCorrection0(), 0.0, 0.0,
    HeadingCorrectionZ() };

  float result[4];
  QuaternionMultiply(quat_corection, quat_, result);
  quat_[0] = result[0];
  quat_[1] = result[1];
  quat_[2] = result[2];
  quat_[3] = result[3];
  QuaternionNormalizingFilter(quat_);
}

// -----------------------------------------------------------------------------
void ResetAttitude(void)
{
  reset_attitude_ = 1;
}

// -----------------------------------------------------------------------------
float * UpdateGravtiyInBody(const float quat[4], float g_b[3])
{
  g_b[X_BODY_AXIS] = 2.0 * (quat[1] * quat[3] - quat[0] * quat[2]);
  g_b[Y_BODY_AXIS] = 2.0 * (quat[2] * quat[3] + quat[0] * quat[1]);
  g_b[Z_BODY_AXIS] = 2.0 * (quat[0] * quat[0] + quat[3] * quat[3]) - 1.0;

  return g_b;
}

// -----------------------------------------------------------------------------
float * UpdateQuaternion(float quat[4], const float angular_rate[3], float dt)
{
  float dpqr[3];
  Vector3Scale(angular_rate, 0.5 * dt, dpqr);

  float d_quat[4];
  d_quat[0] = -dpqr[0] * quat[1] - dpqr[1] * quat[2] - dpqr[2] * quat[3];
  d_quat[1] =  dpqr[0] * quat[0] - dpqr[1] * quat[3] + dpqr[2] * quat[2];
  d_quat[2] =  dpqr[0] * quat[3] + dpqr[1] * quat[0] - dpqr[2] * quat[1];
  d_quat[3] = -dpqr[0] * quat[2] + dpqr[1] * quat[1] + dpqr[2] * quat[0];

  quat[0] += d_quat[0];
  quat[1] += d_quat[1];
  quat[2] += d_quat[2];
  quat[3] += d_quat[3];

  return quat;
}

// -----------------------------------------------------------------------------
void EulerAnglesFromQuaternion(const float quat[4], float * phi, float * theta,
  float * psi)
{
  *phi = atan2(2.0 * (quat[0] * quat[1] + quat[2] * quat[3]), 1.0 - 2.0
    * (quat[1] * quat[1] + quat[2] * quat[2]));
  *theta = asin(2.0 * (quat[0] * quat[2] - quat[1] * quat[3]));
  *psi = HeadingFromQuaternion(quat);
}

// -----------------------------------------------------------------------------
float HeadingFromQuaternion(const float quat[4])
{
  return atan2(2.0 * quat[0] * quat[3] + quat[1] * quat[2], 1.0 - 2.0
    * (quat[2] * quat[2] + quat[3] * quat[3]));
}


// =============================================================================
// Private functions:

static float * CorrectQuaternionWithAccelerometer(float quat[4])
{
  // Assume that the accelerometer measures ONLY the resistance to gravity
  // (opposite the gravity vector). The direction of rotation that takes the
  // body from predicted to estimated gravity is (-accelerometer x g_b_ x). This
  // is equivalent to (g_b_ x accelerometer). Form a corrective quaternion from
  // this rotation.
  float quat_c[4] = { 1.0, 0.0, 0.0, 0.0 };
  Vector3Cross(g_b_, AccelerationVector(), &quat_c[1]);
  quat_c[1] *= 0.5 * ACCELEROMETER_CORRECTION_GAIN;
  quat_c[2] *= 0.5 * ACCELEROMETER_CORRECTION_GAIN;
  quat_c[3] *= 0.5 * ACCELEROMETER_CORRECTION_GAIN;

  // Apply the correction to the attitude quaternion.
  float result[4];
  QuaternionMultiply(quat, quat_c, result);
  quat[0] = result[0];
  quat[1] = result[1];
  quat[2] = result[2];
  quat[3] = result[3];

  return quat;
}

// -----------------------------------------------------------------------------
static void HandleAttitudeReset(void)
{
  quat_[0] = -AccelerationVector()[Z_BODY_AXIS];
  quat_[1] = -AccelerationVector()[Y_BODY_AXIS];
  quat_[2] = AccelerationVector()[X_BODY_AXIS];
  quat_[3] = 0.0;
  quat_[0] += QuaternionNorm(quat_);
  QuaternionNormalize(quat_);

  reset_attitude_ = 0;
}
