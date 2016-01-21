// This file computes motor commands to achieve inner-loop attitude and thrust
// control. The controller was designed to take the following commands:
//   - thrust
//   - heading rate
//   - direction of gravity in the body frame

// The structure of the attitude controller is state-feedback with model-based
// integral action as described in:
//   C. Raabe, S. Suzuki. “Model-Based Integral Action for Multicopters.” Asia
//   Pacific International Symposium on Aerospace Technology, 2015.


#include "control.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>

#include "adc.h"
#include "attitude.h"
#include "custom_math.h"
#include "eeprom.h"
#include "main.h"
#include "motors.h"
#include "quaternion.h"
#include "sbus.h"
#include "state.h"
#include "uart.h"
#include "vector.h"


// =============================================================================
// Private data:

#define CONTROL_MOTORS_IDLE (40)
#define MAX_G_B_CMD (sin(M_PI / 3.0))
#define CMD_MARGIN (20)
#define MIN_CMD (64)
#define MAX_CMD (1840)
#define MAX_ATTITUDE_RATE (M_PI / 2.0)
#define MAX_HEADING_RATE (M_PI / 2.0)

// Computed constants.
static float actuation_inverse_[MAX_MOTORS][4];
static float attitude_error_limit_, heading_error_limit_;
static float attitude_integral_limit_, heading_integral_limit_;
static float k_p_dot_, k_p_, k_phi_, k_phi_int_, k_r_, k_psi_, k_psi_int_;
static float k_sbus_to_thrust_, min_thrust_cmd_, max_thrust_cmd_;

// Variables for control.
static float angular_cmd_[3] = { 0 };
static float attitude_integral_[3] = { 0.0, 0.0, 0.0 };
static float heading_cmd_ = 0.0;
static float quat_cmd_[4];  // Target attitude in quaternion
static float quat_model_[4] = { 1.0, 0.0, 0.0, 0.0 };
static uint16_t setpoints_[MAX_MOTORS] = { 0 };

// Variables for the observer.
static float p_kalman_ = 0.0, p_dot_kalman_ = 0.0, p_dot_bias_ = 0.0;
static float q_kalman_ = 0.0, q_dot_kalman_ = 0.0, q_dot_bias_ = 0.0;


// =============================================================================
// Private function declarations:

static void CommandsFromSticks(float g_b_cmd[2], float * heading_cmd,
  float * heading_rate_cmd, float * thrust_cmd);
static void FormAngularCommand(const float quat_cmd[4],
  const float * heading_rate_cmd, const float attitude_integral[3],
  float angular_cmd[3]);
static void QuaternionFromGravityAndHeadingCommand(const float g_b_cmd[2],
  const float * heading_cmd, float quat_cmd[4]);
static void UpdateAttitudeModel(const float quat_cmd[4],
  const float * heading_rate_cmd, float quat_model[4]);
// static void UpdateIntegrals(const float quat_model[4],
//   float attitude_integral[3]);
static void UpdateKalmanFilter(const float angular_cmd[3]);


// =============================================================================
// Accessors:

float AngularCommand(enum BodyAxes axis)
{
  return angular_cmd_[axis];
}

// -----------------------------------------------------------------------------
const float * AttitudeIntegralVector(void)
{
  return attitude_integral_;
}

// -----------------------------------------------------------------------------
float HeadingCommand(void)
{
  return heading_cmd_;
}

// -----------------------------------------------------------------------------
float KalmanP(void)
{
  return p_kalman_;
}

// -----------------------------------------------------------------------------
float KalmanPDot(void)
{
  return p_dot_kalman_;
}

// -----------------------------------------------------------------------------
float KalmanQ(void)
{
  return q_kalman_;
}

// -----------------------------------------------------------------------------
float KalmanQDot(void)
{
  return q_dot_kalman_;
}

// -----------------------------------------------------------------------------
uint16_t MotorSetpoint(uint8_t n)
{
  return setpoints_[n];
}

// -----------------------------------------------------------------------------
const float * QuatCommandVector(void)
{
  return quat_cmd_;
}

// -----------------------------------------------------------------------------
const float * QuatModelVector(void)
{
  return quat_model_;
}


// =============================================================================
// Public functions:

void ControlInit(void)
{
  eeprom_read_block((void*)actuation_inverse_,
    (const void*)&eeprom.actuation_inverse[0][0], sizeof(actuation_inverse_));

  // TODO: remove these temporary initializations and replace with EEPROM.
  k_p_dot_ = 1.45425059244632;
  k_p_ = 25.1167298526154;
  k_phi_ = 100.0;
  k_phi_int_ = 1.33;

  k_r_ = 4.37792274833246;
  k_psi_ = 7.071067812;
  k_psi_int_ = 0.5;

  // Compute the limit on the attitude error given the rate limit.
  attitude_error_limit_ = MAX_ATTITUDE_RATE * k_p_ / k_phi_;

  // Set the heading error limit to half the attitude error so heading error
  // won't saturate the attitude error.
  heading_error_limit_ = 0.5 * attitude_error_limit_;
/*
  // Compute the thrust limits that give the margin necessary to guarantee
  // that the maximum attitude command is achievable.
  min_thrust_cmd_ = 0;
  max_thrust_cmd_ = FLT_MAX;
  float max_attitude_cmd = attitude_error_limit_ * k_phi_;
  for (uint8_t i = NMotors(); i--; )
  {
    float temp = 1.0 / actuation_inverse_[i][3];
    float min = (float)MIN_CMD * temp;
    float max = (float)MAX_CMD * temp;
    temp *= max_attitude_cmd * sqrt(square(actuation_inverse_[i][0]) +
      square(actuation_inverse_[i][1]));
    min_thrust_cmd_ = FloatMax(min + temp, min_thrust_cmd_);
    max_thrust_cmd_ = FloatMin(max - temp, max_thrust_cmd_);
  }
*/
  min_thrust_cmd_ = 100;
  max_thrust_cmd_ = 1740;

  k_sbus_to_thrust_ = (max_thrust_cmd_ - min_thrust_cmd_) / (2.0 * SBUS_MAX);

  attitude_integral_limit_ = 0.5 * MAX_G_B_CMD;
  heading_integral_limit_ = 0.5 * MAX_HEADING_RATE;
}

// -----------------------------------------------------------------------------
void Control(void)
{
  float g_b_cmd[2];  // Target x and y components of the gravity vector in body
  float heading_rate_cmd, thrust_cmd;

  // TODO: add routines to form commands from an external source.
  // Derive a target attitude from the position of the sticks.
  CommandsFromSticks(g_b_cmd, &heading_cmd_, &heading_rate_cmd, &thrust_cmd);
  QuaternionFromGravityAndHeadingCommand(g_b_cmd, &heading_cmd_, quat_cmd_);

  // Update the integral paths.
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !! TEMPRORAIRLY REMOVED INTEGRALS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // UpdateIntegrals(quat_model_, attitude_integral_);

  // Update the pitch and roll Kalman filters before recomputing the command.
  UpdateKalmanFilter(angular_cmd_);

  // Compute a new attitude acceleration command.
  FormAngularCommand(quat_cmd_, &heading_rate_cmd, attitude_integral_,
    angular_cmd_);

  int16_t limit = FloatToS16(thrust_cmd * 2.0);
  if (limit > MAX_CMD) limit = MAX_CMD;
  for (uint8_t i = NMotors(); i--; )
    setpoints_[i] = (uint16_t)S16Limit(FloatToS16(thrust_cmd
      + VectorDot(angular_cmd_, actuation_inverse_[i])), MIN_CMD, limit);

  if (MotorsRunning())
    for (uint8_t i = NMotors(); i--; ) SetMotorSetpoint(i, setpoints_[i]);
  else if (MotorsStarting())
    for (uint8_t i = NMotors(); i--; ) SetMotorSetpoint(i, CONTROL_MOTORS_IDLE);
  else
    for (uint8_t i = NMotors(); i--; ) SetMotorSetpoint(i, 0);

  // TxMotorSetpoints();

  // Update the model for the next time step.
  UpdateAttitudeModel(quat_cmd_, &heading_rate_cmd, quat_model_);
}

// -----------------------------------------------------------------------------
void SetActuationInverse(float actuation_inverse[MAX_MOTORS][4])
{
  eeprom_update_block((const void*)actuation_inverse,
    (void*)&eeprom.actuation_inverse[0][0], sizeof(actuation_inverse_));
  ControlInit();
}


// =============================================================================
// Private functions:

// This function converts the position of the R/C transmitter sticks to a thrust
// command, heading rate command, and command corresponding to the direction of
// gravity in the body frame. Note that commanding the direction of gravity is
// similar to a pitch and roll command, but is actually closer to the intended
// result and has the benefit of a direct correspondence with linear
// acceleration.
static void CommandsFromSticks(float g_b_cmd[2], float * heading_cmd,
  float * heading_rate_cmd, float * thrust_cmd)
{
  if (SBusStale())
  {
    g_b_cmd[X_BODY_AXIS] = 0.0;
    g_b_cmd[Y_BODY_AXIS] = 0.0;
    *heading_rate_cmd = 0.0;
  }
  else
  {
    g_b_cmd[X_BODY_AXIS] = (float)SBusPitch() * MAX_G_B_CMD / (float)SBUS_MAX;
    g_b_cmd[Y_BODY_AXIS] = -(float)SBusRoll() * MAX_G_B_CMD / (float)SBUS_MAX;
    *heading_rate_cmd = -(float)SBusYaw() * MAX_HEADING_RATE / (float)SBUS_MAX;
  }

  // Scale back the commanded components at the corners (optional).
  g_b_cmd[X_BODY_AXIS] -= g_b_cmd[X_BODY_AXIS] * (float)abs(SBusRoll())
    * MAX_G_B_CMD / (4.0 * (float)SBUS_MAX);
  g_b_cmd[Y_BODY_AXIS] -= g_b_cmd[Y_BODY_AXIS] * (float)abs(SBusPitch())
    * MAX_G_B_CMD / (4.0 * (float)SBUS_MAX);

  // Integrate the yaw stick to get a heading command.
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // if (MotorsRunning())
  //   *heading_cmd += *heading_rate_cmd * DT;
  // else
    *heading_cmd = HeadingAngle();
  while (*heading_cmd > M_PI) *heading_cmd -= 2.0 * M_PI;
  while (*heading_cmd < -M_PI) *heading_cmd += 2.0 * M_PI;

  // Compute the thrust command.
  *thrust_cmd = (float)(SBusThrust() + SBUS_MAX) * k_sbus_to_thrust_
    + min_thrust_cmd_;
}

// -----------------------------------------------------------------------------
// This function converts the combination of the x and y components of a unit
// vector corresponding to the commanded direction of gravity in the body frame
// and a heading commanded to a target quaternion. Note that, in the interest of
// computational efficiency, the heading of the resulting quaternion may not
// exactly match heading command for attitude commands that are far from level.
// Also note that the heading error is saturated in this step to avoid an overly
// large yawing command.
static void QuaternionFromGravityAndHeadingCommand(const float g_b_cmd[2],
  const float * heading_cmd, float quat_cmd[4])
{
  // Compute the z component of the gravity vector command.
  float g_b_cmd_z = sqrt(1.0 - square(g_b_cmd[X_BODY_AXIS])
    - square(g_b_cmd[Y_BODY_AXIS]));

  // Form a quaternion from these components (z component is 0).
  float temp1 = 0.5 + 0.5 * g_b_cmd_z;
  float quat_g_b_cmd_0 = sqrt(temp1);
  float temp2 = 1.0 / (2.0 * quat_g_b_cmd_0);
  float quat_g_b_cmd_x = g_b_cmd[Y_BODY_AXIS] * temp2;
  float quat_g_b_cmd_y = -g_b_cmd[X_BODY_AXIS] * temp2;

  // Determine the (approximate) heading of this command for removal (optional).
  float heading_from_g_b_cmd = (quat_g_b_cmd_x * quat_g_b_cmd_y) / (temp1
    + square(quat_g_b_cmd_x) - 0.5);

  // Limit the heading error.
  float heading_err = FloatLimit(WrapToPlusMinusPi(*heading_cmd
    - HeadingAngle()), -heading_error_limit_, heading_error_limit_);

  // Make a second quaternion for the commanded heading minus the residual
  // heading from the gravity vector command (x and y components are zero).
  temp1 = (HeadingAngle() + heading_err - heading_from_g_b_cmd) * 0.5;
  float quat_heading_cmd_0 = cos(temp1);
  float quat_heading_cmd_z = sin(temp1);

  // Combine the quaternions (heading rotation first) to form the final
  // quaternion command.
  quat_cmd[0] = quat_g_b_cmd_0 * quat_heading_cmd_0;
  quat_cmd[1] = quat_g_b_cmd_x * quat_heading_cmd_0 + quat_g_b_cmd_y
    * quat_heading_cmd_z;
  quat_cmd[2] = -quat_g_b_cmd_x * quat_heading_cmd_z + quat_g_b_cmd_y
    * quat_heading_cmd_0;
  quat_cmd[3] = quat_g_b_cmd_0 * quat_heading_cmd_z;
}

// -----------------------------------------------------------------------------
// This function computes the rotation vector that will take the multicopter
// from the current attitude to the commanded attitude (represented by
// quat_cmd).
static float * AttitudeError(const float quat_cmd[4], const float quat[4],
  float attitude_error[3])
{
  // Find the quaternion that goes from the current to the command.
  float quat_err[4];
  QuaternionMultiplyInverse(quat_cmd, quat, quat_err);

  // Make sure the error is represented as a positive quaternion.
  if (quat_err[0] < 0)
  {
    // quat_err[0] = -quat_err[0];  // Unused
    quat_err[1] = -quat_err[1];
    quat_err[2] = -quat_err[2];
    quat_err[3] = -quat_err[3];
  }

  // The last 3 elements of the resulting quaternion represent a vector along
  // axis of the rotation that will take the aircraft to the desired attitude
  // (if the first element is positive). The magnitude of this vector is equal
  // to the sine of half the rotation. Assume that the angle is small enough to
  // ignore the sine, so just double the magnitude.
  attitude_error[0] = 2.0 * quat_err[1];
  attitude_error[1] = 2.0 * quat_err[2];
  attitude_error[2] = 2.0 * quat_err[3];

  return attitude_error;
}

// -----------------------------------------------------------------------------
static void FormAngularCommand(const float quat_cmd[4],
  const float * heading_rate_cmd, const float attitude_integral[3],
  float angular_cmd[3])
{
  float attitude_error[3];
  AttitudeError(quat_cmd, Quat(), attitude_error);

  // Saturate the error.
  float attitude_error_norm = VectorNorm(attitude_error);
  if (attitude_error_norm > attitude_error_limit_)
    VectorGain(attitude_error, attitude_error_limit_
      / attitude_error_norm, attitude_error);

  // Transform the yaw rate command into the body axis. Note that yaw rate
  // happens to occur along the gravity vector, so yaw rate command is a simple
  // scalar multiplication of the gravity vector.
  float rate_cmd[3];
  VectorGain(GravityInBodyVector(), *heading_rate_cmd, rate_cmd);

  // Apply the control gains.
  angular_cmd[X_BODY_AXIS] = k_phi_ * (attitude_error[X_BODY_AXIS]
    + attitude_integral[X_BODY_AXIS]) + k_p_ * (rate_cmd[X_BODY_AXIS]
    - p_kalman_) + k_p_dot_ * -p_dot_kalman_;
  angular_cmd[Y_BODY_AXIS] = k_phi_ * (attitude_error[Y_BODY_AXIS]
    + attitude_integral[Y_BODY_AXIS]) + k_p_ * (rate_cmd[Y_BODY_AXIS]
    - q_kalman_) + k_p_dot_ * -q_dot_kalman_;
  angular_cmd[Z_BODY_AXIS] = k_psi_ * (attitude_error[Z_BODY_AXIS]
    + attitude_integral[Z_BODY_AXIS]) + k_r_ * (rate_cmd[Z_BODY_AXIS]
    - AngularRate(Z_BODY_AXIS));
}

// -----------------------------------------------------------------------------
// This function updates a Kalman filter that combines the angular acceleration
// that is expected given the motor commands and the derivative of the measured
// angular rate. This step is necessary because the derivative of the measured
// angular rate is extremely noisy, resulting in large commands. Note that the
// process and measurement noise covariances are assumed to be constant and the
// Kalman gains are pre-computed for the resulting stead-state error covariance.
static void UpdateKalmanFilter(const float angular_cmd[3])
{
  // Past values for derivatives.
  static float p_pv = 0.0, q_pv = 0.0;

  // TODO: replace these precomputed constants with EEPROM.
  const float kA11 = 0.924848813216205, kA13 = 0.00751511867837952;
  const float kA21 = 0.00751511867837952, kA23 = 2.97381321620483e-05;
  const float kB11 = 0.0751511867837952, kB21 = 0.000297381321620483;
  const float kK[3][2] = {
    { 0.00913677925111990, 7.27850351648910 },
    { 0.000222122299697543, 0.306277877559922 },
    { 0.235922172480771, 144.569834075959 } };

  // Prediction.
  p_kalman_ += kA21 * p_dot_kalman_ + kA23 * p_dot_bias_
    + kB21 * angular_cmd[X_BODY_AXIS];
  p_dot_kalman_ = kA11 * p_dot_kalman_ + kA13 * p_dot_bias_
    + kB11 * angular_cmd[X_BODY_AXIS];

  q_kalman_ += kA21 * q_dot_kalman_ + kA23 * q_dot_bias_
    + kB21 * angular_cmd[Y_BODY_AXIS];
  q_dot_kalman_ = kA11 * q_dot_kalman_ + kA13 * q_dot_bias_
    + kB11 * angular_cmd[Y_BODY_AXIS];

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

// -----------------------------------------------------------------------------
// This function updates the attitude model that is used for the model-based
// integral action.
static void UpdateAttitudeModel(const float quat_cmd[4],
  const float * heading_rate_cmd, float quat_model[4])
{
  // Compute the error between the model's attitude and the command.
  float attitude_error[3];
  AttitudeError(quat_cmd, quat_model, attitude_error);

  // Distribute the heading rate command to the axes.
  float g_b[3], rate_cmd[3];
  UpdateGravtiyInBody(quat_model, g_b);
  VectorGain(g_b, *heading_rate_cmd, rate_cmd);

  float angular_rate[3];
  static float delay[3][2] = { 0 };

  // TODO: replace these precomputed constants with EEPROM.
  const float np[2] = { 0.000286210197180126, 0.000268485917540157 };
  const float dp[2] = { -1.81159070436859, 0.825522856832319 };
  angular_rate[X_BODY_AXIS] = DirectForm2ZeroB0(k_p_ * rate_cmd[X_BODY_AXIS]
    + k_phi_ * attitude_error[X_BODY_AXIS], np, dp, delay[X_BODY_AXIS]);
  angular_rate[Y_BODY_AXIS] = DirectForm2ZeroB0(k_p_ * rate_cmd[Y_BODY_AXIS]
    + k_phi_ * attitude_error[Y_BODY_AXIS], np, dp, delay[Y_BODY_AXIS]);

  const float nr[2] = { 0.0118819811047824, -0.0113035400156885 };
  const float dr[2] = { -1.89561493354849, 0.897611422809053 };
  angular_rate[Z_BODY_AXIS] = DirectForm2ZeroB0(k_r_ * rate_cmd[Z_BODY_AXIS]
    + k_psi_ * attitude_error[Z_BODY_AXIS], nr, dr, delay[Z_BODY_AXIS]);

  UpdateQuaternion(quat_model, angular_rate, DT);
  QuaternionNormalizingFilter(quat_model);
}
/*
// -----------------------------------------------------------------------------
// This function updates the error integrals.
static void UpdateIntegrals(const float quat_model[4],
  float attitude_integral[3])
{
  // Compute the error between the actual and model attitudes.
  float attitude_error[3];
  AttitudeError(quat_model, Quat(), attitude_error);

  attitude_integral[X_BODY_AXIS] = FloatLimit(attitude_integral[X_BODY_AXIS]
    + k_phi_int_ * attitude_error[X_BODY_AXIS] * DT, -attitude_integral_limit_,
    attitude_integral_limit_);
  attitude_integral[Y_BODY_AXIS] = FloatLimit(attitude_integral[Y_BODY_AXIS]
    + k_phi_int_ * attitude_error[Y_BODY_AXIS] * DT, -attitude_integral_limit_,
    attitude_integral_limit_);
  attitude_integral[Z_BODY_AXIS] = FloatLimit(attitude_integral[Z_BODY_AXIS]
    + k_psi_int_ * attitude_error[Z_BODY_AXIS] * DT, -heading_integral_limit_,
    heading_integral_limit_);
}
*/