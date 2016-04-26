// This file computes motor commands to achieve inner-loop attitude and thrust
// control. The controller was designed to take the following commands:
//   - thrust
//   - heading rate
//   - direction of gravity in the body frame

// The structure of the attitude controller is state-feedback with model-based
// integral action as described in:
//   C. Raabe, S. Suzuki. "Model-Based Integral Action for Multicopters." Asia
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
#include "vector.h"
#include "vertical_speed.h"


// =============================================================================
// Private data:

#define MIN_CMD (64)
#define MAX_CMD (1840)
#define MIN_THRUST_CMD (100)
#define MAX_THRUST_CMD (1400)
#define MAX_G_B_CMD (sin(M_PI / 3.0))
// TODO: unify this with limits_.heading_rate
#define MAX_HEADING_RATE (M_PI / 4.0)
#define MAX_VERTICAL_SPEED (1.0)
#define MIN_TRANSIT_SPEED (0.1)
#define MAX_TRANSIT_SPEED (2.0)
#define STARTING_SETPOINT (40)

// Computed constants.
static float actuation_inverse_[MAX_MOTORS][4];

static struct FeedbackGains {
  float p_dot;
  float p;
  float phi;
  float r;
  float psi;
  float psi_integral;
  float w_dot;
  float w;
  float x_dot;
  float x;
  float x_integral;
  float z;
  float z_integral;
} feedback_gains_ = { 0 };

static struct Limits {
  float heading_rate;
  float heading_error;
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

static struct Model {
  float angular_acceleration[3];
  float angular_rate[3];
  float eular_angles[3];
  float vertical_acceleration;
  float velocity[3];
  float position[3];
} model_ = { 0 };

static struct PositionControlState {
  float position_integral[3];
  float heading_integral;
  float target_baro_altitude;
  int16_t hover_thrust_stick;
  uint8_t control_mode_pv;
} position_control_state_ = { 0 };

// TODO: make some of these local to Control()
static float angular_cmd_[3] = { 0.0 };
static float heading_cmd_ = 0.0, thrust_cmd_ = 0.0;
static float nav_g_b_cmd_[2] = { 0.0 }, nav_thrust_cmd_ = 0.0;
static float quat_cmd_[4];  // Target attitude in quaternion


static float k_motor_lag_ = 0.0;
static float k_speed_to_position_error_ = 0.0, k_position_error_to_speed_ = 0.0,
  k_horiszontal_limit_to_vertical_limit = 0.0,
  k_vertical_limit_to_horizontal_limit = 0.0;
static uint16_t setpoints_[MAX_MOTORS] = { 0 };


// =============================================================================
// Private function declarations:

static void CommandsForPositionControl(const struct FeedbackGains * k,
  const struct Limits * limit, float g_b_cmd[2], float * heading_cmd,
  float * heading_rate_cmd, float * thrust_cmd, struct Model * model,
  struct PositionControlState * state);
static void CommandsFromSticks(float g_b_cmd[2], float * heading_cmd,
  float * heading_rate_cmd, float * thrust_cmd);
static void FormAngularCommand(const float quat_cmd[4],
  float heading_rate_cmd, const struct KalmanState * kalman,
  const struct FeedbackGains * k, float angular_cmd[3]);
static void QuaternionFromGravityAndHeadingCommand(const float g_b_cmd[2],
  const struct Limits * limit, float heading_cmd, float quat_cmd[4]);
static void ResetModel(const float position[3], const float velocity[3],
  struct Model * m);
static void UpdateKalmanFilter(const float angular_cmd[3],
  const struct KalmanCoeffiecients * k, struct KalmanState * x);
static void UpdateModel(const float position_cmd[3],
  const float velocity_cmd[3], const struct FeedbackGains * k,
  float position_error_limit, struct Model * m);

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
float NavThrustCommand(void)
{
  return nav_thrust_cmd_;
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
#if defined BI_OCTO
  // control proportion: 0.400000
  feedback_gains_.p_dot = 5.532231665e-01;
  feedback_gains_.p = 1.503704565e+01;
  feedback_gains_.phi = 5.590657197e+01;
  feedback_gains_.r = 2.664374986e+00;
  feedback_gains_.psi = 3.731358603e+00;

  feedback_gains_.psi_integral = 1.742256838e+00 * DT / feedback_gains_.psi;

  feedback_gains_.x_dot = 0.18;
  feedback_gains_.x = 0.135;
  feedback_gains_.x_integral = 0.045 * DT;

  feedback_gains_.w_dot = 0.0;
  feedback_gains_.w = 2.0;
  feedback_gains_.z = 1.5;
  feedback_gains_.z_integral = 0.45 * DT * actuation_inverse_[0][3];

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

  k_motor_lag_ = 1.0 / 0.07;
#elif defined SMALL_QUAD
  // control proportion: 0.500000
  feedback_gains_.p_dot = +2.690295082e+00;
  feedback_gains_.p = +5.941758937e+01;
  feedback_gains_.phi = +3.674012659e+02;
  feedback_gains_.r = +4.921024667e+00;
  feedback_gains_.psi = +1.786057092e+01;

  feedback_gains_.x_dot = 0.18;
  feedback_gains_.x = 0.135;
  feedback_gains_.x_integral = 0.045 * DT;

  feedback_gains_.w_dot = 5.091813030e-03;
  feedback_gains_.w = 4.407621675e+00;
  feedback_gains_.z = 7.422916434e+00;
  feedback_gains_.z_integral = 4.854441330e+00 * DT * actuation_inverse_[0][3];

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

  k_motor_lag_ = 1.0 / 0.1;
#else
  // control proportion: 0.500000
  feedback_gains_.p_dot = 1.147582995e+00;
  feedback_gains_.p = 2.874714450e+01;
  feedback_gains_.phi = 1.477784660e+02;
  feedback_gains_.r = 5.019351450e+00;
  feedback_gains_.psi = 1.493714521e+01;

  feedback_gains_.psi_integral = 1.195915077e+01 * DT / feedback_gains_.psi;

  feedback_gains_.x_dot = 0.18;
  feedback_gains_.x = 0.135;
  feedback_gains_.x_integral = 0.045 * DT;

  feedback_gains_.w_dot = 0.0;
  feedback_gains_.w = 2.0;
  feedback_gains_.z = 1.5;
  feedback_gains_.z_integral = 0.45 * DT * actuation_inverse_[0][3];

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

  k_motor_lag_ = 1.0 / 0.07;
#endif

  // TODO: Handle this actuation inverse in a smarter way.
  // Limit heading and heading rate error to 25% of control authority.
  limits_.heading_rate = 0.25 * (MAX_CMD - MIN_CMD) / (feedback_gains_.r
    * fabs(actuation_inverse_[0][2]));
  limits_.heading_error = 0.25 * (MAX_CMD - MIN_CMD) / (feedback_gains_.psi
    * fabs(actuation_inverse_[0][2]));

  k_speed_to_position_error_ = feedback_gains_.x_dot / feedback_gains_.x;
  k_position_error_to_speed_ = 1.0 / k_speed_to_position_error_;
  k_horiszontal_limit_to_vertical_limit = feedback_gains_.w / feedback_gains_.z
    / k_speed_to_position_error_;
  k_vertical_limit_to_horizontal_limit = 1.0
    / k_horiszontal_limit_to_vertical_limit;
}

// -----------------------------------------------------------------------------
void Control(void)
{
  float g_b_cmd[2];  // Target x and y components of the gravity vector in body
  float heading_rate_cmd;

  // TODO: add routines to form commands from an external source.
  // Derive a target attitude from the position of the sticks.
  CommandsFromSticks(g_b_cmd, &heading_cmd_, &heading_rate_cmd, &thrust_cmd_);
  CommandsForPositionControl(&feedback_gains_, &limits_, nav_g_b_cmd_,
    &heading_cmd_, &heading_rate_cmd, &nav_thrust_cmd_, &model_,
    &position_control_state_);

  g_b_cmd[0] += nav_g_b_cmd_[0];
  g_b_cmd[1] += nav_g_b_cmd_[1];
  thrust_cmd_ += nav_thrust_cmd_;

  QuaternionFromGravityAndHeadingCommand(g_b_cmd, &limits_, heading_cmd_,
    quat_cmd_);

  // Update the pitch and roll Kalman filters before recomputing the command.
  UpdateKalmanFilter(angular_cmd_, &kalman_coefficients_, &kalman_state_);

  // Compute a new attitude acceleration command.
  FormAngularCommand(quat_cmd_, heading_rate_cmd, &kalman_state_,
    &feedback_gains_, angular_cmd_);

  int16_t limit = FloatToS16(thrust_cmd_ * 2.0);
  if (limit > MAX_CMD) limit = MAX_CMD;
  for (uint8_t i = NMotors(); i--; )
    setpoints_[i] = (uint16_t)S16Limit(FloatToS16(thrust_cmd_
      + Vector3Dot(angular_cmd_, actuation_inverse_[i])), MIN_CMD, limit);

  if (MotorsRunning())
    for (uint8_t i = NMotors(); i--; ) SetMotorSetpoint(i, setpoints_[i]);
  else if (MotorsStarting())
    for (uint8_t i = NMotors(); i--; ) SetMotorSetpoint(i, STARTING_SETPOINT);
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
static void CommandsForPositionControl(const struct FeedbackGains * k,
  const struct Limits * limit, float g_b_cmd[2], float * heading_cmd,
  float * heading_rate_cmd, float * thrust_cmd, struct Model * model,
  struct PositionControlState * state)
{
  float position[3] = { 0.0 }, velocity[3] = { 0.0 };
  float position_cmd[3] = { 0.0 }, velocity_cmd[3] = { 0.0 };
  float position_error_limit = MIN_TRANSIT_SPEED;
  int16_t thrust_offset = 0;

  switch (ControlMode())
  {
    case CONTROL_MODE_NAV:
    {
      if ((NavStatus() != 1) || NavStale()) return;  // Do not update

      // Copy volatile data to local memory.
      Vector3Copy((const float *)PositionVector(), position);
      Vector3Copy((const float *)VelocityVector(), velocity);
      Vector3Copy((const float *)TargetPositionVector(), position_cmd);

      // Set the position error limit according to the transit speed. (Also,
      // make sure the transit speed is sane.)
      position_error_limit = FloatLimit(TransitSpeed(), MIN_TRANSIT_SPEED,
        MAX_TRANSIT_SPEED) * k_speed_to_position_error_;

      // Form a heading rate command depending on the heading error.
      float heading_rate_limit = FloatLimit(HeadingRate(), 0.02,
        limit->heading_rate);
      *heading_rate_cmd += FloatSLimit(WrapToPlusMinusPi(TargetHeading()
        - *heading_cmd + state->heading_integral) / DT, heading_rate_limit);
      *heading_cmd += *heading_rate_cmd * DT;
      state->heading_integral += FloatSLimit(WrapToPlusMinusPi(TargetHeading()
        - HeadingAngle()) * k->psi_integral, M_PI / 12);  // Limited to 15-deg
      break;
    }
    case CONTROL_MODE_BARO_ALTITUDE:
    {
      if (state->control_mode_pv != CONTROL_MODE_BARO_ALTITUDE)
      {
        state->hover_thrust_stick = SBusThrust();
        state->target_baro_altitude = DeltaPressureAltitude();
      }

      position[D_WORLD_AXIS] = -DeltaPressureAltitude();
      velocity[D_WORLD_AXIS] = -VerticalSpeed();

      velocity_cmd[D_WORLD_AXIS] = -(SBusThrust() - state->hover_thrust_stick)
        * (MAX_VERTICAL_SPEED / (float)SBUS_MAX);

      state->target_baro_altitude += -velocity_cmd[D_WORLD_AXIS] * DT;
      position_cmd[D_WORLD_AXIS] = -state->target_baro_altitude;

      position_error_limit = 1.5 * MAX_VERTICAL_SPEED
        * k_speed_to_position_error_;  // 50% margin

      // Offset the thrust command so that vertical speed commands don't affect
      // the raw thrust command.
      thrust_offset = state->hover_thrust_stick - SBusThrust();
      break;
    }
    case CONTROL_MODE_MANUAL:
    default:
      break;
  }

  // Reset the integrator when the mode changes.
  if (ControlMode() != state->control_mode_pv)
  {
    state->position_integral[N_WORLD_AXIS] = 0.0;
    state->position_integral[E_WORLD_AXIS] = 0.0;
    state->position_integral[D_WORLD_AXIS] = 0.0;
    ResetModel(position, velocity, model);
  }

  // Integrate the difference with the model.
  UpdateModel(position_cmd, velocity_cmd, k, position_error_limit, model);
  state->position_integral[N_WORLD_AXIS] = FloatSLimit(
    state->position_integral[N_WORLD_AXIS] + (model->position[N_WORLD_AXIS]
    - position[N_WORLD_AXIS]) * k->x_integral, 0.25 * MAX_G_B_CMD);
  state->position_integral[E_WORLD_AXIS] = FloatSLimit(
    state->position_integral[E_WORLD_AXIS] + (model->position[E_WORLD_AXIS]
    - position[E_WORLD_AXIS]) * k->x_integral, 0.25 * MAX_G_B_CMD);
  state->position_integral[D_WORLD_AXIS] = FloatSLimit(
    state->position_integral[D_WORLD_AXIS] + (model->position[D_WORLD_AXIS]
    - position[D_WORLD_AXIS]) * k->z_integral, 0.15
    * (MAX_THRUST_CMD - MIN_THRUST_CMD));

  float position_error[3], velocity_error[3];
  Vector3Subtract(position_cmd, position, position_error);
  Vector3Subtract(velocity_cmd, velocity, velocity_error);

  // If the position error limit is exceeded, then switch to velocity control.
  position_error[D_WORLD_AXIS] *= k_vertical_limit_to_horizontal_limit;
  float position_error_norm = Vector3Norm(position_error);
  if (position_error_norm > position_error_limit)
  {
    Vector3ScaleSelf(position_error, position_error_limit
      / position_error_norm);
    position_error[D_WORLD_AXIS] *= k_horiszontal_limit_to_vertical_limit;
    Vector3AddToSelf(position_error, model->position);
    Vector3SubtractFromSelf(position_error, position);
  }
  else
  {
    position_error[D_WORLD_AXIS] *= k_horiszontal_limit_to_vertical_limit;
  }

  // Limit the navigation command to half the maximum manual command.
  float a_w_cmd[2];  // acceleration command in world N-E plane
  a_w_cmd[N_WORLD_AXIS] = FloatSLimit(
    + k->x_dot * velocity_error[N_WORLD_AXIS]
    + k->x * position_error[N_WORLD_AXIS],
    0.5 * MAX_G_B_CMD)
    + state->position_integral[N_WORLD_AXIS];
  a_w_cmd[E_WORLD_AXIS] = FloatSLimit(
    + k->x_dot * velocity_error[E_WORLD_AXIS]
    + k->x * position_error[E_WORLD_AXIS],
    0.5 * MAX_G_B_CMD)
    + state->position_integral[E_WORLD_AXIS];

  // Rotate the world commands to the body (assuming small pitch/roll angles).
  float cos_heading = cos(HeadingAngle()), sin_heading = sin(HeadingAngle());
  g_b_cmd[X_BODY_AXIS] = cos_heading * a_w_cmd[N_WORLD_AXIS] + sin_heading
    * a_w_cmd[E_WORLD_AXIS];
  g_b_cmd[Y_BODY_AXIS] = cos_heading * a_w_cmd[E_WORLD_AXIS] - sin_heading
    * a_w_cmd[N_WORLD_AXIS];

  // TODO: do this actuation inverse in a smarter way.
  *thrust_cmd = FloatSLimit(actuation_inverse_[0][3] * (
    + k->w_dot * -(-VerticalAcceleration())
    + k->w * velocity_error[D_WORLD_AXIS]
    + k->z * position_error[D_WORLD_AXIS]),
    0.25 * (MAX_THRUST_CMD - MIN_THRUST_CMD))
    + state->position_integral[Z_BODY_AXIS] + thrust_offset;

  state->control_mode_pv = ControlMode();
}

// -----------------------------------------------------------------------------
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

  // Compute the thrust command.
  const float k_sbus_to_thrust_ = (float)(MAX_THRUST_CMD - MIN_THRUST_CMD)
    / (2.0 * (float)SBUS_MAX);
  *thrust_cmd = (float)(SBusThrust() + SBUS_MAX) * k_sbus_to_thrust_
    + MIN_THRUST_CMD;
}

// -----------------------------------------------------------------------------
static void FormAngularCommand(const float quat_cmd[4],
  float heading_rate_cmd, const struct KalmanState * kalman,
  const struct FeedbackGains * k, float angular_cmd[3])
{
  float attitude_error[3];
  AttitudeError(quat_cmd, Quat(), attitude_error);

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
// This function converts the combination of the x and y components of a unit
// vector corresponding to the commanded direction of gravity in the body frame
// and a heading commanded to a target quaternion. Note that, in the interest of
// computational efficiency, the heading of the resulting quaternion may not
// exactly match heading command for attitude commands that are far from level.
// Also note that the heading error is saturated in this step to avoid an overly
// large yawing command.
static void QuaternionFromGravityAndHeadingCommand(const float g_b_cmd[2],
  const struct Limits * limit, float heading_cmd, float quat_cmd[4])
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
  float heading_error = FloatSLimit(WrapToPlusMinusPi(heading_cmd
    - HeadingAngle()), limit->heading_error);

  // Make a second quaternion for the commanded heading minus the residual
  // heading from the gravity vector command (x and y components are zero).
  float temp = (HeadingAngle() + heading_error - heading_from_g_b_cmd) * 0.5;
  float quat_heading_cmd_0 = cos(temp);
  float quat_heading_cmd_z = sin(temp);

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
static void ResetModel(const float position[3], const float velocity[3],
  struct Model * m)
{
  m->angular_acceleration[0] = 0.0;
  m->angular_acceleration[1] = 0.0;
  m->angular_acceleration[2] = 0.0;
  m->angular_rate[0] = 0.0;
  m->angular_rate[1] = 0.0;
  m->angular_rate[2] = 0.0;
  m->eular_angles[0] = 0.0;
  m->eular_angles[1] = 0.0;
  m->eular_angles[2] = 0.0;
  m->vertical_acceleration = 0.0;
  Vector3Copy(velocity, m->velocity);
  Vector3Copy(position, m->position);
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

// -----------------------------------------------------------------------------
// Simple linear decoupled model of position in NED frame with zero heading.
static void UpdateModel(const float position_cmd[3],
  const float velocity_cmd[3], const struct FeedbackGains * k,
  float position_error_limit, struct Model * m)
{
  float position_error[3], velocity_error[3];
  Vector3Subtract(position_cmd, m->position, position_error);
  Vector3Subtract(velocity_cmd, m->velocity, velocity_error);

  // TODO: the following assumes that this limit affects horizontal and vertical
  // motion in the same way, which may not be true with different gains!!!
  // Limit the position error to give the desired transit speed.
  position_error[D_WORLD_AXIS] *= k_vertical_limit_to_horizontal_limit;
  float position_error_norm = Vector3Norm(position_error);
  if (position_error_norm > position_error_limit)
  {
    Vector3ScaleSelf(position_error, position_error_limit
      / position_error_norm);
  }
  position_error[D_WORLD_AXIS] *= k_horiszontal_limit_to_vertical_limit;

  // Form horizontal acceleration commands.
  float a_w_cmd[3];  // acceleration command in world NED frame
  a_w_cmd[N_WORLD_AXIS] =
    + k->x_dot * velocity_error[N_WORLD_AXIS]
    + k->x * position_error[N_WORLD_AXIS];
  a_w_cmd[E_WORLD_AXIS] =
    + k->x_dot * velocity_error[E_WORLD_AXIS]
    + k->x * position_error[E_WORLD_AXIS];
  a_w_cmd[D_WORLD_AXIS] =
    + k->w_dot * -m->vertical_acceleration
    + k->w * velocity_error[D_WORLD_AXIS]
    + k->z * position_error[D_WORLD_AXIS];

  // Apply the control gains.
  float angular_cmd[2];
  angular_cmd[X_BODY_AXIS] =
    + k->p_dot * -m->angular_acceleration[X_BODY_AXIS]
    + k->p * -m->angular_rate[X_BODY_AXIS]
    + k->phi * (a_w_cmd[E_WORLD_AXIS] - m->eular_angles[X_BODY_AXIS]);
  angular_cmd[Y_BODY_AXIS] =
    + k->p_dot * -m->angular_acceleration[Y_BODY_AXIS]
    + k->p * -m->angular_rate[Y_BODY_AXIS]
    + k->phi * (-a_w_cmd[N_WORLD_AXIS] - m->eular_angles[Y_BODY_AXIS]);

  float p_dot_dot = k_motor_lag_ * (angular_cmd[X_BODY_AXIS]
    - m->angular_acceleration[X_BODY_AXIS]);
  m->angular_acceleration[X_BODY_AXIS] += p_dot_dot * DT;
  m->angular_rate[X_BODY_AXIS] += m->angular_acceleration[X_BODY_AXIS] * DT;
  m->eular_angles[X_BODY_AXIS] += m->angular_rate[X_BODY_AXIS] * DT;
  m->velocity[E_WORLD_AXIS] += m->eular_angles[X_BODY_AXIS]
    * GRAVITY_ACCELERATION * DT;
  m->position[E_WORLD_AXIS] += m->velocity[E_WORLD_AXIS] * DT;

  float q_dot_dot = k_motor_lag_ * (angular_cmd[Y_BODY_AXIS]
    - m->angular_acceleration[Y_BODY_AXIS]);
  m->angular_acceleration[Y_BODY_AXIS] += q_dot_dot * DT;
  m->angular_rate[Y_BODY_AXIS] += m->angular_acceleration[Y_BODY_AXIS] * DT;
  m->eular_angles[Y_BODY_AXIS] += m->angular_rate[Y_BODY_AXIS] * DT;
  m->velocity[N_WORLD_AXIS] -= m->eular_angles[Y_BODY_AXIS]
    * GRAVITY_ACCELERATION * DT;
  m->position[N_WORLD_AXIS] += m->velocity[N_WORLD_AXIS] * DT;

  // TODO: make the direction of vertical_acceleration consistent with
  // vertical_speed (from pressure altitude)
  float w_dot_dot = k_motor_lag_ * (a_w_cmd[D_WORLD_AXIS]
    - m->vertical_acceleration);
  m->vertical_acceleration += w_dot_dot * DT;
  m->velocity[D_WORLD_AXIS] += m->vertical_acceleration * DT;
  m->position[D_WORLD_AXIS] += m->velocity[D_WORLD_AXIS] * DT;
}
