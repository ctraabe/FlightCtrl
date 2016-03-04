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

#include <math.h>
#include <stdlib.h>

#include "adc.h"
#include "attitude.h"
#include "custom_math.h"
#include "eeprom.h"
#include "main.h"
#include "motors.h"
#include "nav_comms.h"
#include "quaternion.h"
#include "pressure_altitude.h"
#include "sbus.h"
#include "state.h"
#include "uart.h"
#include "vector.h"
#include "vertical_speed.h"


// =============================================================================
// Private data:

#define CONTROL_MOTORS_IDLE (40)
#define MAX_G_B_CMD (sin(M_PI / 3.0))
#define CMD_MARGIN (20)
#define MIN_CMD (64)
#define MIN_THRUST_CMD (100)
#define MAX_THRUST_CMD (1400)
#define MAX_CMD (1840)
#define MAX_ATTITUDE_RATE (4.0 * M_PI)
#define MAX_HEADING_RATE (M_PI / 2.0)

// Computed constants.
static float actuation_inverse_[MAX_MOTORS][4];
// static float k_sbus_to_thrust_, min_thrust_cmd_, max_thrust_cmd_;

static struct FeedbackGains {
  float p_dot;
  float p;
  float phi;
  float r;
  float psi;
  float w_dot;
  float w;
  float x_dot;
  float x;
  float z;
  float z_int;
} feedback_gains_ = { 0 };

static struct Limits {
  float attitude_error;
  float heading_error;
  float position_error;
  float velocity_error;
  float vertical_speed_error;
  float altitude_error;
  float z_integral;
} limits_ = { 0 };

static struct KalmanCoeffiecients {
  float A11;
  float A13;
  float A21;
  float A23;
  float B11;
  float B21;
  float K[3][2];
} kalman_coefficients_ = { 0 };

static struct KalmanState {
  float p_dot;
  float p;
  float p_dot_bias;
  float q_dot;
  float q;
  float q_dot_bias;
  float p_pv;
  float q_pv;
} kalman_state_ = { 0 };

static float angular_cmd_[3] = { 0 };
static float heading_cmd_ = 0.0, thrust_cmd_ = 0.0;
static float nav_g_b_cmd_[2];
static float quat_cmd_[4];  // Target attitude in quaternion
static uint16_t setpoints_[MAX_MOTORS] = { 0 };

// TODO: remove
static float thrust_cmd_to_z_integral = 0.0;


// =============================================================================
// Private function declarations:

static void CommandsFromSticks(float g_b_cmd[2], float * heading_cmd,
  float * heading_rate_cmd);
static void FormAngularCommand(const float quat_cmd[4],
  float heading_rate_cmd, const struct KalmanState * kalman,
  const struct FeedbackGains * k, const struct Limits * limits,
  float angular_cmd[3]);
static void QuaternionFromGravityAndHeadingCommand(const float g_b_cmd[2],
  float heading_cmd, float quat_cmd[4]);
static void FormThrustCommand(const struct FeedbackGains * k,
  const struct Limits * limits, float * thrust_cmd);
static void GravityCommandsFromNav(const struct FeedbackGains * k,
  const struct Limits * limits, float g_b_cmd[2]);
static void UpdateKalmanFilter(const float angular_cmd[3],
  const struct KalmanCoeffiecients * k, struct KalmanState * x);

// =============================================================================
// Accessors:

float AngularCommand(enum BodyAxes axis)
{
  return angular_cmd_[axis];
}

// -----------------------------------------------------------------------------
float HeadingCommand(void)
{
  return heading_cmd_;
}

// -----------------------------------------------------------------------------
float KalmanP(void)
{
  return kalman_state_.p;
}

// -----------------------------------------------------------------------------
float KalmanPDot(void)
{
  return kalman_state_.p_dot;
}

// -----------------------------------------------------------------------------
float KalmanQ(void)
{
  return kalman_state_.q;
}

// -----------------------------------------------------------------------------
float KalmanQDot(void)
{
  return kalman_state_.q_dot;
}

// -----------------------------------------------------------------------------
uint16_t MotorSetpoint(uint8_t n)
{
  return setpoints_[n];
}

// -----------------------------------------------------------------------------
const float * NavGBCommand(void)
{
  return nav_g_b_cmd_;
}

// -----------------------------------------------------------------------------
const float * QuatCommandVector(void)
{
  return quat_cmd_;
}

// -----------------------------------------------------------------------------
float ThrustCommand(void)
{
  return thrust_cmd_;
}


// =============================================================================
// Public functions:

void ControlInit(void)
{
  eeprom_read_block((void*)actuation_inverse_,
    (const void*)&eeprom.actuation_inverse[0][0], sizeof(actuation_inverse_));

  // TODO: remove these temporary initializations and replace computations.
#ifdef SMALL_QUAD
  // control proportion: 0.400000
  feedback_gains_.p_dot = 2.756626048e+00;
  feedback_gains_.p = 6.157278112e+01;
  feedback_gains_.phi = 3.875709893e+02;
  feedback_gains_.r = 5.088500555e+00;
  feedback_gains_.psi = 1.927238878e+01;
  feedback_gains_.w_dot = 5.091813030e-03;
  feedback_gains_.w = 4.407621675e+00;
  feedback_gains_.z = 7.422916434e+00;
  feedback_gains_.z_int = 4.854441330e+00;
  kalman_coefficients_.A11 = 9.248488132e-01;
  kalman_coefficients_.A13 = 7.515118678e-03;
  kalman_coefficients_.A21 = 7.515118678e-03;
  kalman_coefficients_.A23 = 2.973813216e-05;
  kalman_coefficients_.B11 = 7.515118678e-02;
  kalman_coefficients_.B21 = 2.973813216e-04;
  kalman_coefficients_.K[0][0] = 9.136779251e-03;
  kalman_coefficients_.K[0][1] = 7.278503516e+00;
  kalman_coefficients_.K[1][0] = 2.221222997e-04;
  kalman_coefficients_.K[1][1] = 3.062778776e-01;
  kalman_coefficients_.K[2][0] = 2.359221725e-01;
  kalman_coefficients_.K[2][1] = 1.445698341e+02;
  feedback_gains_.x_dot = 0.16;
  feedback_gains_.x = 0.1;
#else
  // control proportion: 0.400000
  feedback_gains_.p_dot = 6.125465888e-01;
  feedback_gains_.p = 1.620762355e+01;
  feedback_gains_.phi = 6.256019555e+01;
  feedback_gains_.r = 2.493839316e+00;
  feedback_gains_.psi = 3.316200008e+00;
  feedback_gains_.w_dot = 0.000000000e+00;
  feedback_gains_.w = 2.021170072e+00;
  feedback_gains_.z = 1.500000000e+00;
  feedback_gains_.z_int = 3.500000000e-01;
  kalman_coefficients_.A11 = 8.943955582e-01;
  kalman_coefficients_.A13 = 7.392310928e-03;
  kalman_coefficients_.A21 = 7.392310928e-03;
  kalman_coefficients_.A23 = 2.941323505e-05;
  kalman_coefficients_.B11 = 1.056044418e-01;
  kalman_coefficients_.B21 = 4.201890722e-04;
  kalman_coefficients_.K[0][0] = 7.736180483e-03;
  kalman_coefficients_.K[0][1] = 6.465227478e+00;
  kalman_coefficients_.K[1][0] = 1.973030846e-04;
  kalman_coefficients_.K[1][1] = 2.905144232e-01;
  kalman_coefficients_.K[2][0] = 2.225348506e-01;
  kalman_coefficients_.K[2][1] = 1.470361439e+02;
  feedback_gains_.x_dot = 0.18;
  feedback_gains_.x = 0.135;
#endif

  // Compute the limit on the attitude error given the rate limit.
  limits_.attitude_error = MAX_ATTITUDE_RATE * feedback_gains_.p
    / feedback_gains_.phi;

  // Set the heading error limit to half the attitude error so heading error
  // won't saturate the attitude error.
  limits_.heading_error = 0.5 * limits_.attitude_error;

/*
  // Compute the thrust limits that give the margin necessary to guarantee
  // that the maximum attitude command is achievable.
  // min_thrust_cmd_ = 0;
  // max_thrust_cmd_ = FLT_MAX;
  float max_attitude_cmd = limits_.attitude_error * k_phi_;
  for (uint8_t i = NMotors(); i--; )
  {
    float temp = 1.0 / actuation_inverse_[i][3];
    float min = (float)MIN_CMD * temp;
    float max = (float)MAX_CMD * temp;
    temp *= max_attitude_cmd * sqrt(square(actuation_inverse_[i][0]) +
      square(actuation_inverse_[i][1]));
    // min_thrust_cmd_ = FloatMax(min + temp, min_thrust_cmd_);
    // max_thrust_cmd_ = FloatMin(max - temp, max_thrust_cmd_);
  }

  // k_sbus_to_thrust_ = (max_thrust_cmd_ - min_thrust_cmd_) / (2.0 * SBUS_MAX);
*/

  // NOTE: speed ~ k_x / k_x_dot * positiion_error_limit
  // SO: positiion_error_limit ~ speed * k_x_dot / k_x
  limits_.position_error = 1.0 * feedback_gains_.x_dot / feedback_gains_.x;
  limits_.velocity_error = 2.0 * limits_.position_error;
  limits_.altitude_error = 0.5;
  limits_.vertical_speed_error = 2.5;

  // TODO remove:
  thrust_cmd_to_z_integral = 1.0 / feedback_gains_.z_int
    / actuation_inverse_[0][3];

  limits_.z_integral = (7 * MAX_THRUST_CMD / 8) * thrust_cmd_to_z_integral;
}

// -----------------------------------------------------------------------------
void Control(void)
{
  float g_b_cmd[2];  // Target x and y components of the gravity vector in body
  float heading_rate_cmd;

  // TODO: add routines to form commands from an external source.
  // Derive a target attitude from the position of the sticks.
  CommandsFromSticks(g_b_cmd, &heading_cmd_, &heading_rate_cmd);
  GravityCommandsFromNav(&feedback_gains_, &limits_, nav_g_b_cmd_);
  g_b_cmd[0] += nav_g_b_cmd_[0];
  g_b_cmd[1] += nav_g_b_cmd_[1];
  QuaternionFromGravityAndHeadingCommand(g_b_cmd, heading_cmd_, quat_cmd_);

  // Update the pitch and roll Kalman filters before recomputing the command.
  UpdateKalmanFilter(angular_cmd_, &kalman_coefficients_, &kalman_state_);

  // Compute a new attitude acceleration command.
  FormAngularCommand(quat_cmd_, heading_rate_cmd, &kalman_state_,
    &feedback_gains_, &limits_, angular_cmd_);

  // Compute a thrust command
  FormThrustCommand(&feedback_gains_, &limits_, &thrust_cmd_);

  int16_t limit = FloatToS16(thrust_cmd_ * 2.0);
  if (limit > MAX_CMD) limit = MAX_CMD;
  for (uint8_t i = NMotors(); i--; )
    setpoints_[i] = (uint16_t)S16Limit(FloatToS16(thrust_cmd_
      + Vector3Dot(angular_cmd_, actuation_inverse_[i])), MIN_CMD, limit);

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

// This function computes the rotation vector that will take the multicopter
// from the current attitude to the commanded attitude (represented by
// quat_cmd).
static float * AttitudeError(const float quat_cmd[4], const float quat[4],
  float attitude_error[3])
{
  // Find the quaternion that goes from the current to the command.
  float quat_err[4];
  QuaternionInverseMultiply(quat, quat_cmd, quat_err);

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
// This function converts the position of the R/C transmitter sticks to a thrust
// command, heading rate command, and command corresponding to the direction of
// gravity in the body frame. Note that commanding the direction of gravity is
// similar to a pitch and roll command, but is actually closer to the intended
// result and has the benefit of a direct correspondence with linear
// acceleration.
static void CommandsFromSticks(float g_b_cmd[2], float * heading_cmd,
  float * heading_rate_cmd)
{
  if (SBusStale())
  {
    g_b_cmd[X_BODY_AXIS] = 0.0;
    g_b_cmd[Y_BODY_AXIS] = 0.0;
    *heading_rate_cmd = 0.0;
    // TODO: determine hover thrust setting and set it here, minus a bit
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
  *heading_cmd = WrapToPlusMinusPi(*heading_cmd);
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
  float heading_cmd, float quat_cmd[4])
{
  // Compute the z component of the gravity vector command.
  float g_b_cmd_z = sqrt(1.0 - g_b_cmd[X_BODY_AXIS] * g_b_cmd[X_BODY_AXIS]
    - g_b_cmd[Y_BODY_AXIS] * g_b_cmd[Y_BODY_AXIS]);

  // Form a quaternion from these components (z component is 0).
  float temp1 = 0.5 + 0.5 * g_b_cmd_z;
  float quat_g_b_cmd_0 = sqrt(temp1);
  float temp2 = 1.0 / (2.0 * quat_g_b_cmd_0);
  float quat_g_b_cmd_x = g_b_cmd[Y_BODY_AXIS] * temp2;
  float quat_g_b_cmd_y = -g_b_cmd[X_BODY_AXIS] * temp2;

  // Determine the (approximate) heading of this command for removal (optional).
  float heading_from_g_b_cmd = (quat_g_b_cmd_x * quat_g_b_cmd_y) / (temp1
    + quat_g_b_cmd_x * quat_g_b_cmd_x - 0.5);

  // Limit the heading error.
  // float heading_err = FloatLimit(WrapToPlusMinusPi(heading_cmd
  //   - HeadingAngle()), -heading_error_limit_, heading_error_limit_);

  // Make a second quaternion for the commanded heading minus the residual
  // heading from the gravity vector command (x and y components are zero).
  temp1 = (heading_cmd - heading_from_g_b_cmd) * 0.5;
  // temp1 = (HeadingAngle() + heading_err - heading_from_g_b_cmd) * 0.5;
  float quat_heading_cmd_0 = cos(temp1);
  float quat_heading_cmd_z = sin(temp1);

  // Combine the quaternions (heading rotation first) to form the final
  // quaternion command.
  quat_cmd[0] = quat_heading_cmd_0 * quat_g_b_cmd_0;
  quat_cmd[1] = quat_heading_cmd_0 * quat_g_b_cmd_x - quat_heading_cmd_z
    * quat_g_b_cmd_y;
  quat_cmd[2] = quat_heading_cmd_0 * quat_g_b_cmd_y + quat_heading_cmd_z
    * quat_g_b_cmd_x;
  quat_cmd[3] = quat_heading_cmd_z * quat_g_b_cmd_0;
}

// -----------------------------------------------------------------------------
static void FormAngularCommand(const float quat_cmd[4],
  float heading_rate_cmd, const struct KalmanState * kalman,
  const struct FeedbackGains * k, const struct Limits * limits,
  float angular_cmd[3])
{
  float attitude_error[3];
  AttitudeError(quat_cmd, Quat(), attitude_error);

  // Saturate the error.
  float attitude_error_norm = Vector3Norm(attitude_error);
  if (attitude_error_norm > limits->attitude_error)
    Vector3Scale(attitude_error, limits->attitude_error
      / attitude_error_norm, attitude_error);

  // Transform the yaw rate command into the body axis. Note that yaw rate
  // happens to occur along the gravity vector, so yaw rate command is a simple
  // scalar multiplication of the gravity vector.
  float rate_cmd[3];
  Vector3Scale(GravityInBodyVector(), heading_rate_cmd, rate_cmd);

  // Apply the control gains.
  angular_cmd[X_BODY_AXIS] =
    + k->p_dot * -kalman->p_dot
    + k->p * (rate_cmd[X_BODY_AXIS] - AngularRate(X_BODY_AXIS))
    + k->phi * attitude_error[X_BODY_AXIS];
  angular_cmd[Y_BODY_AXIS] =
    + k->p_dot * -kalman->q_dot
    + k->p * (rate_cmd[Y_BODY_AXIS] - AngularRate(Y_BODY_AXIS))
    + k->phi * attitude_error[Y_BODY_AXIS];
  angular_cmd[Z_BODY_AXIS] =
    + k->r * (rate_cmd[Z_BODY_AXIS] - AngularRate(Z_BODY_AXIS))
    + k->psi * attitude_error[Z_BODY_AXIS];
}

// -----------------------------------------------------------------------------
static void GravityCommandsFromNav(const struct FeedbackGains * k,
  const struct Limits * limits, float g_b_cmd[2])
{
  if ((NavModeRequest() == NAV_MODE_OFF) || (NavMode() == NAV_MODE_OFF))
  {
    g_b_cmd[0] = 0.0;
    g_b_cmd[1] = 0.0;
    return;
  }

  if (NavStatus() != 1) return;

  // Limit the position error.
  float x_i_error = FloatLimit(-PositionVector()[0], -limits->position_error,
    limits->position_error);
  float y_i_error = FloatLimit(-PositionVector()[1], -limits->position_error,
    limits->position_error);

  // Limit the velocity error.
  float vx_i_error = FloatLimit(-VelocityVector()[0], -limits->velocity_error,
    limits->velocity_error);
  float vy_i_error = FloatLimit(-VelocityVector()[1], -limits->velocity_error,
    limits->velocity_error);

  float a_i_cmd[2];  // acceleration command in inertial x-y plane
  a_i_cmd[0] = FloatLimit(k->x_dot * vx_i_error + k->x * x_i_error, -0.4, 0.4);
  a_i_cmd[1] = FloatLimit(k->x_dot * vy_i_error + k->x * y_i_error, -0.4, 0.4);

  // TODO: do not assume that the following does not contribute to heading.
  float cos_heading = cos(HeadingAngle()), sin_heading = sin(HeadingAngle());
  g_b_cmd[X_BODY_AXIS] = cos_heading * a_i_cmd[0] + sin_heading * a_i_cmd[1];
  g_b_cmd[Y_BODY_AXIS] = cos_heading * a_i_cmd[1] - sin_heading * a_i_cmd[0];
}

// -----------------------------------------------------------------------------
static void FormThrustCommand(const struct FeedbackGains * k,
  const struct Limits * limits, float * thrust_cmd)
{
  // TODO: make these inputs
  static int16_t hover_thrust_stick = 0;
  static float z_integral = 0.0;
  static float z_cmd = 0.0;
  static uint8_t altitude_control_pv = 0, pressure_control_pv = 0;

  // TODO: remove
  static float thrust_cmd_pv = 0.0;
  uint8_t nav_active = (NavModeRequest() != NAV_MODE_OFF)
    && (NavMode() != NAV_MODE_OFF);

  if (!AltitudeControlActive())
  {
    const float k_sbus_to_thrust_ = (float)(MAX_THRUST_CMD - MIN_THRUST_CMD)
      / (2.0 * (float)SBUS_MAX);
    *thrust_cmd = (float)(SBusThrust() + SBUS_MAX) * k_sbus_to_thrust_
      + MIN_THRUST_CMD;
  }
  else
  {
    float w, z;

    // TODO: compute these
    if (!altitude_control_pv)
    {
      hover_thrust_stick = SBusThrust();
      z_integral = thrust_cmd_to_z_integral * thrust_cmd_pv;
    }

    if (nav_active)
    {
      if (NavStatus() != 1) return;
      z_cmd = 0;
      z = PositionVector()[2];
      w = VelocityVector()[2];
    }
    else
    {
      if (!pressure_control_pv) z_cmd = -DeltaPressureAltitude();
      z = -DeltaPressureAltitude();
      w = -VerticalSpeed();
    }

    float w_cmd = (float)(hover_thrust_stick - SBusThrust())
      / (float)(SBUS_MAX) * 0.25;
    float w_error = FloatLimit(w_cmd - w, -limits->vertical_speed_error,
      limits->vertical_speed_error);
    float z_error = FloatLimit(z_cmd - z, -limits->altitude_error,
      limits->altitude_error);

    // TODO: do this actuation inverse multiplication in a smarter way!
    *thrust_cmd = FloatLimit(actuation_inverse_[0][3] * (
      + k->w_dot * -(-VerticalAcceleration())
      + k->w * w_error
      + k->z * z_error
      + k->z_int * z_integral),
      MIN_THRUST_CMD, MAX_THRUST_CMD);

    // TODO: do this elsewhere
    z_integral = FloatLimit(z_integral + z_error * DT, limits->z_integral, 0.0);
    z_cmd += w_cmd * DT;
  }

  altitude_control_pv = AltitudeControlActive();
  pressure_control_pv = AltitudeControlActive() && !nav_active;
  thrust_cmd_pv = *thrust_cmd;
}

// -----------------------------------------------------------------------------
// This function updates a Kalman filter that combines the angular acceleration
// that is expected given the motor commands and the derivative of the measured
// angular rate. This step is necessary because the derivative of the measured
// angular rate is extremely noisy, resulting in large commands. Note that the
// process and measurement noise covariances are assumed to be constant and the
// Kalman gains are pre-computed for the resulting stead-state error covariance.
static void UpdateKalmanFilter(const float angular_cmd[3],
  const struct KalmanCoeffiecients * k, struct KalmanState * x)
{
  // Prediction.
  x->p += k->A21 * x->p_dot + k->A23 * x->p_dot_bias
    + k->B21 * angular_cmd[X_BODY_AXIS];
  x->p_dot = k->A11 * x->p_dot + k->A13 * x->p_dot_bias
    + k->B11 * angular_cmd[X_BODY_AXIS];

  x->q += k->A21 * x->q_dot + k->A23 * x->q_dot_bias
    + k->B21 * angular_cmd[Y_BODY_AXIS];
  x->q_dot = k->A11 * x->q_dot + k->A13 * x->q_dot_bias
    + k->B11 * angular_cmd[Y_BODY_AXIS];

  // Correction.
  float p_dot_err = (AngularRate(X_BODY_AXIS) - x->p_pv) / DT - x->p_dot;
  float p_err = AngularRate(X_BODY_AXIS) - x->p;
  x->p_dot += k->K[0][0] * p_dot_err + k->K[0][1] * p_err;
  x->p += k->K[1][0] * p_dot_err + k->K[1][1] * p_err;
  x->p_dot_bias += k->K[2][0] * p_dot_err + k->K[2][1] * p_err;

  float q_dot_err = (AngularRate(Y_BODY_AXIS) - x->q_pv) / DT - x->q_dot;
  float q_err = AngularRate(Y_BODY_AXIS) - x->q;
  x->q_dot += k->K[0][0] * q_dot_err + k->K[0][1] * q_err;
  x->q += k->K[1][0] * q_dot_err + k->K[1][1] * q_err;
  x->q_dot_bias += k->K[2][0] * q_dot_err + k->K[2][1] * q_err;

  // Save past values
  x->p_pv = AngularRate(X_BODY_AXIS);
  x->q_pv = AngularRate(Y_BODY_AXIS);
}
