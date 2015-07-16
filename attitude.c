#include "attitude.h"

#include <math.h>

#include "adc.h"
#include "quaternion.h"
#include "vector.h"


// =============================================================================
// Private data:

#define ACCELEROMETER_CORRECTION_GAIN (0.001)

static float quat_[4] = { 1.0, 0.0, 0.0, 0.0 }, g_b_[3] = { 0.0, 0.0, 1.0 };
static float heading_angle_ = 0.0;
static float p_kalman_ = 0.0, p_dot_kalman_ = 0.0, p_dot_bias_ = 0.0;
static float q_kalman_ = 0.0, q_dot_kalman_ = 0.0, q_dot_bias_ = 0.0;


// =============================================================================
// Private function declarations:

static void CorrectQuaternionWithAccelerometer(void);
static void UpdateGravtiyInBody(void);
static void UpdateQuaternion(void);


// =============================================================================
// Accessors:

float * GravityInBodyVector(void)
{
  return g_b_;
}

// -----------------------------------------------------------------------------
float HeadingAngle(void)
{
  return heading_angle_;
}

// -----------------------------------------------------------------------------
float * Quat(void)
{
  return quat_;
}


// =============================================================================
// Public functions:

void ResetAttitude(void)
{
  quat_[0] = -AccelerationVector()[Z_BODY_AXIS];
  quat_[1] = -AccelerationVector()[Y_BODY_AXIS];
  quat_[2] = AccelerationVector()[X_BODY_AXIS];
  quat_[3] = 0.0;
  quat_[0] += QuaternionNorm(quat_);
  QuaternionNormalize(quat_);
}

// -----------------------------------------------------------------------------
void UpdateAttitude(void)
{
  UpdateQuaternion();
  UpdateGravtiyInBody();
  CorrectQuaternionWithAccelerometer();
  QuaternionNormalizingFilter(quat_);
  heading_angle_ = atan2(2.0 * quat_[0] * quat_[3] + quat_[1] * quat_[2],
    1.0 - 2.0 * (square(quat_[2]) + square(quat_[3])));
}

// -----------------------------------------------------------------------------
void UpdateKalmanFilter(float p_dot_cmd, float q_dot_cmd)
{
  // Past values for derivatives.
  static float p_pv = 0.0, q_pv = 0.0;

  // Precomputed constants.
  const float kA11 = 0.94925, kA13 = 0.0076125;
  const float kA21 = 0.0076125, kA23 = 3.0e-5;
  const float kB11 = 0.05075, kB21 = 0.0002;
  const float kK[3][2] = { { 1.0, 1.0 }, { 1.0, 1.0 }, { 1.0, 1.0 } };

  // Prediction.
  p_kalman_ += kA21 * p_dot_kalman_ + kA23 * p_dot_bias_ + kB21 * p_dot_cmd;
  p_dot_kalman_ = kA11 * p_dot_kalman_ + kA13 * p_dot_bias_ + kB11 * p_dot_cmd;

  q_kalman_ += kA21 * q_dot_kalman_ + kA23 * q_dot_bias_ + kB21 * q_dot_cmd;
  q_dot_kalman_ = kA11 * q_dot_kalman_ + kA13 * q_dot_bias_ + kB11 * q_dot_cmd;

  // Correction.
  float p_dot_err = (AngularRate(X_BODY_AXIS) - p_pv) / DT - p_dot_kalman_;
  float p_err = AngularRate(X_BODY_AXIS) - p_kalman_;
  p_dot_kalman_ += kK[0][0] * p_dot_err + kK[0][1] * p_err;
  p_kalman_ += kK[1][0] * p_dot_err + kK[1][1] * p_err;
  p_dot_bias_ += kK[2][0] * p_dot_err + kK[2][1] * p_err;

  float q_dot_err = (AngularRate(Y_BODY_AXIS) - q_pv) / DT - q_dot_kalman_;
  float q_err = AngularRate(Y_BODY_AXIS) - q_kalman_;
  q_dot_kalman_ += kK[0][0] * q_dot_err + kK[0][1] * q_err;
  q_kalman_ += kK[1][0] * q_dot_err + kK[1][1] * q_err;
  q_dot_bias_ += kK[2][0] * q_dot_err + kK[2][1] * q_err;

  // Save past values for derivatives.
  p_pv = AngularRate(X_BODY_AXIS);
  q_pv = AngularRate(Y_BODY_AXIS);
}


// =============================================================================
// Private functions:

static void CorrectQuaternionWithAccelerometer(void)
{
  // Assume that the accelerometer measures ONLY the resistance to gravity (
  // opposite the gravity vector). The direction of rotation that takes the body
  // from predicted to estimated gravity is (-accelerometer x g_b_ x). This is
  // equivalent to (g_b_ x accelerometer). Form a corrective quaternion from
  // this rotation.
  float quat_c[4] = { 1.0, 0.0, 0.0, 0.0 };
  VectorCross(g_b_, AccelerationVector(), &quat_c[1]);
  quat_c[1] *= 0.5 * ACCELEROMETER_CORRECTION_GAIN;
  quat_c[2] *= 0.5 * ACCELEROMETER_CORRECTION_GAIN;
  quat_c[3] *= 0.5 * ACCELEROMETER_CORRECTION_GAIN;

  // Apply the correction to the attitude quaternion.
  float result[4];
  QuaternionMultiply(quat_, quat_c, result);
  quat_[0] = result[0];
  quat_[1] = result[1];
  quat_[2] = result[2];
  quat_[3] = result[3];
}

// -----------------------------------------------------------------------------
static void UpdateGravtiyInBody(void)
{
  g_b_[0] = 2.0 * (quat_[1] * quat_[3] - quat_[0] * quat_[2]);
  g_b_[1] = 2.0 * (quat_[2] * quat_[3] + quat_[0] * quat_[1]);
  g_b_[2] = 2.0 * (quat_[0] * quat_[0] + quat_[3] * quat_[3]) - 1.0;
}

// -----------------------------------------------------------------------------
static void UpdateQuaternion(void)
{
  float dpqr[3];
  VectorGain(AngularRateVector(), 0.5 * DT, dpqr);

  float d_quat[4];
  d_quat[0] = -dpqr[0] * quat_[1] - dpqr[1] * quat_[2] - dpqr[2] * quat_[3];
  d_quat[1] =  dpqr[0] * quat_[0] - dpqr[1] * quat_[3] + dpqr[2] * quat_[2];
  d_quat[2] =  dpqr[0] * quat_[3] + dpqr[1] * quat_[0] - dpqr[2] * quat_[1];
  d_quat[3] = -dpqr[0] * quat_[2] + dpqr[1] * quat_[1] + dpqr[2] * quat_[0];

  quat_[0] += d_quat[0];
  quat_[1] += d_quat[1];
  quat_[2] += d_quat[2];
  quat_[3] += d_quat[3];
}
