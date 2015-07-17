#include "control.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>

#include "adc.h"
#include "attitude.h"
#include "eeprom.h"
#include "motors.h"
#include "mymath.h"
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
#define MAX_ATTITUDE_RATE (1.0)
#define MAX_HEADING_RATE (1.0)

static float actuation_inverse_[MAX_MOTORS][4];
static float attitude_cmd_[3] = { 0 };
static float attitude_error_limit_, heading_error_limit_, attitude_cmd_[3];
static float k_phi_, k_p_, k_p_dot_, k_psi_, k_r_;
static float k_sbus_to_thrust_, min_thrust_cmd_, max_thrust_cmd_;
static float p_kalman_ = 0.0, p_dot_kalman_ = 0.0, p_dot_bias_ = 0.0;
static float q_kalman_ = 0.0, q_dot_kalman_ = 0.0, q_dot_bias_ = 0.0;
static uint16_t setpoints_[MAX_MOTORS] = { 0 };


// =============================================================================
// Private function declarations:

static float * AttitudeCommand(float g_b_cmd[2], float * heading_cmd,
  float * heading_rate_cmd, float attitude_cmd[3]);
static void CommandsFromSticks(float g_b_cmd[2], float * heading_cmd,
  float * heading_rate_cmd, float * thrust_cmd);
static void UpdateKalmanFilter(void);


// =============================================================================
// Accessors:

float AttitudeCmd(enum BodyAxes axis)
{
  return attitude_cmd_[axis];
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


// =============================================================================
// Public functions:

void ControlInit(void)
{
  eeprom_read_block((void*)actuation_inverse_,
    (const void*)&eeprom.actuation_inverse[0][0], sizeof(actuation_inverse_));

  // TODO: remove these temporary initializations and replace with EEPROM.
  k_phi_ = 100.0;
  k_p_ = 28.00407596;
  k_p_dot_ = 2.066141351;
  k_psi_ = 7.071067812;
  k_r_ = 4.884168229;
  // k_r_dot_ = -0.02029653396;

  // Compute the limit on the attitude error given the rate limit.
  attitude_error_limit_ = MAX_ATTITUDE_RATE * k_p_ / k_phi_;

  // Set the heading error limit to half the attitude error so heading error
  // won't saturate the attitude error.
  heading_error_limit_ = attitude_error_limit_ * 0.5;

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

  k_sbus_to_thrust_ = (max_thrust_cmd_ - min_thrust_cmd_) / (2.0 * SBUS_MAX);
}

// -----------------------------------------------------------------------------
void Control(void)
{
  float g_b_cmd[2];  // Target x and y components of the gravity vector in body
  static float heading_cmd = 0.0;
  float heading_rate_cmd, thrust_cmd;

  // TODO: add routines to form commands from an external source.
  // Derive a target attitude from the position of the sticks.
  CommandsFromSticks(g_b_cmd, &heading_cmd, &heading_rate_cmd, &thrust_cmd);

  // Update the pitch and roll Kalman filters.
  UpdateKalmanFilter();

  // Computer a new attitude acceleration command.
  AttitudeCommand(g_b_cmd, &heading_cmd, &heading_rate_cmd, attitude_cmd_);

  for (uint8_t i = NMotors(); i--; )
    setpoints_[i] = (uint16_t)S16Limit(FloatToS16(thrust_cmd
      * actuation_inverse_[i][3] + VectorDot(attitude_cmd_,
      actuation_inverse_[i])), MIN_CMD, MAX_CMD);

  if (MotorsRunning())
    for (uint8_t i = NMotors(); i--; ) SetMotorSetpoint(i, setpoints_[i]);
  else if (MotorsStarting())
    for (uint8_t i = NMotors(); i--; ) SetMotorSetpoint(i, CONTROL_MOTORS_IDLE);
  else
    for (uint8_t i = NMotors(); i--; ) SetMotorSetpoint(i, 0);

  TxMotorSetpoints();
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

static float * AttitudeCommand(float g_b_cmd[2], float * heading_cmd,
  float * heading_rate_cmd, float attitude_cmd[3])
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
  float psi_from_g_b_cmd = (quat_g_b_cmd_x * quat_g_b_cmd_y) / (temp1
    + square(quat_g_b_cmd_x) - 0.5);

  // Limit the heading error.
  float heading_err = FloatLimit(WrapToPlusMinusPi(*heading_cmd
    - HeadingAngle()), -heading_error_limit_, heading_error_limit_);

  // Make a second quaternion for the commanded heading minus the residual
  // heading from the gravity vector command (x and y components are zero).
  temp1 = (HeadingAngle() + heading_err - psi_from_g_b_cmd) * 0.5;
  float quat_heading_cmd_0 = cos(temp1);
  float quat_heading_cmd_z = sin(temp1);

  // Combine the quaternions (heading rotation first) to form the final
  // quaternion command.
  float quat_cmd[4];
  quat_cmd[0] = quat_g_b_cmd_0 * quat_heading_cmd_0;
  quat_cmd[1] = quat_g_b_cmd_x * quat_heading_cmd_0 + quat_g_b_cmd_y
    * quat_heading_cmd_z;
  quat_cmd[2] = -quat_g_b_cmd_x * quat_heading_cmd_z + quat_g_b_cmd_y
    * quat_heading_cmd_0;
  quat_cmd[3] = quat_g_b_cmd_0 * quat_heading_cmd_z;

  // Find the quaternion that goes from the current to the command.
  float quat_err[4], quat_inv[4];
  QuaternionMultiply(quat_cmd, QuaternionInverse(Quat(), quat_inv), quat_err);

  // The last 3 elements of the resulting quaternion represent a vector along
  // axis of the rotation that will take the aircraft to the desired attitude
  // (if the first element is positive). The magnitude of this vector is equal
  // to the sine of half the rotation. Assume that the angle is small enough to
  // ignore the sine, so just double the magnitude.
  if (quat_err[0] < 0) QuaternionInverse(quat_err, quat_err);
  float attitude_error[3] = { 2.0 * quat_err[1], 2.0 * quat_err[2],
    2.0 * quat_err[3] };

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
  attitude_cmd[X_BODY_AXIS] = k_phi_ * attitude_error[X_BODY_AXIS]
    + k_p_ * (rate_cmd[X_BODY_AXIS] - p_kalman_) + k_p_dot_ * -p_dot_kalman_;
  attitude_cmd[Y_BODY_AXIS] = k_phi_ * attitude_error[Y_BODY_AXIS]
    + k_p_ * (rate_cmd[Y_BODY_AXIS] - q_kalman_) + k_p_dot_ * -q_dot_kalman_;
  attitude_cmd[Z_BODY_AXIS] = k_psi_ * attitude_error[Z_BODY_AXIS]
    + k_r_ * (rate_cmd[Z_BODY_AXIS] - AngularRate(Z_BODY_AXIS));

  return attitude_cmd;
}

// -----------------------------------------------------------------------------
void CommandsFromSticks(float g_b_cmd[2], float * heading_cmd,
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
  if (MotorsRunning())
    *heading_cmd += *heading_rate_cmd * DT;
  else
    *heading_cmd = HeadingAngle();
  while (*heading_cmd > M_PI) *heading_cmd -= 2.0 * M_PI;
  while (*heading_cmd < -M_PI) *heading_cmd += 2.0 * M_PI;

  // Compute the thrust command.
  *thrust_cmd = (float)(SBusThrust() + SBUS_MAX) * k_sbus_to_thrust_
    + min_thrust_cmd_;
}

// -----------------------------------------------------------------------------
static void UpdateKalmanFilter(void)
{
  // Past values for derivatives.
  static float p_pv = 0.0, q_pv = 0.0;

  // Precomputed constants.
  const float kA11 = 0.949249759400175, kA13 = 0.00761253608997371;
  const float kA21 = 0.00761253608997372, kA23 = 2.99945865039428e-05;
  const float kB11 = 0.0507502405998248, kB21 = 0.000199963910026285;
  const float kK[3][2] = {
    { 0.0104471900215786, 8.00809367719265 },
    { 0.000244387624426045, 0.319481296836928 },
    { 0.247271564219495, 142.430886162968 } };

  // Prediction.
  p_kalman_ += kA21 * p_dot_kalman_ + kA23 * p_dot_bias_
    + kB21 * attitude_cmd_[X_BODY_AXIS];
  p_dot_kalman_ = kA11 * p_dot_kalman_ + kA13 * p_dot_bias_
    + kB11 * attitude_cmd_[X_BODY_AXIS];

  q_kalman_ += kA21 * q_dot_kalman_ + kA23 * q_dot_bias_
    + kB21 * attitude_cmd_[Y_BODY_AXIS];
  q_dot_kalman_ = kA11 * q_dot_kalman_ + kA13 * q_dot_bias_
    + kB11 * attitude_cmd_[Y_BODY_AXIS];

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
