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
  float phi_int;
  float r;
  float psi;
  float psi_int;
  float w_dot;
  float w;
  float z;
  float z_int;
} feedback_gains_ = { 0 };

static struct Limits {
  float attitude_error;
  float heading_error;
  float attitude_integral;
  float heading_integral;
  float altitude_error;
  float altitude_integral;
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

static struct AttitudeModelCoefficients {
  float p_q[2][2];
  float r[2][2];
} attitude_model_coefficients_ = { 0 };

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

static struct AttitudeModel {
  float quat[4];
  float delays[3][2];
} attitude_model_ = { 0 };

static float attitude_integral_[3] = { 0.0, 0.0, 0.0 };

static float angular_cmd_[3] = { 0 };
static float heading_cmd_ = 0.0;
static float quat_cmd_[4];  // Target attitude in quaternion
static float quat_model_[4] = { 1.0, 0.0, 0.0, 0.0 };
static uint16_t setpoints_[MAX_MOTORS] = { 0 };

// TODO: remove
static float thrust_cmd_to_z_integral = 0.0;


// =============================================================================
// Private function declarations:

static void CommandsFromSticks(float g_b_cmd[2], float * heading_cmd,
  float * heading_rate_cmd);
static void FormAngularCommand(const float quat_cmd[4],
  float heading_rate_cmd, const struct KalmanState * kalman,
  const float attitude_integral[3], const struct FeedbackGains * k,
  const struct Limits * limits, float angular_cmd[3]);
static void QuaternionFromGravityAndHeadingCommand(const float g_b_cmd[2],
  float heading_cmd, float quat_cmd[4]);
static float ThrustCommand(const struct FeedbackGains * k,
  const struct Limits * limits);
static void UpdateAttitudeModel(const float quat_cmd[4], float heading_rate_cmd,
  const struct FeedbackGains * k, const struct AttitudeModelCoefficients * c,
  struct AttitudeModel * model);
static void UpdateIntegrals(const struct AttitudeModel * model,
  const struct Limits * limits, float attitude_integral[3]);
static void UpdateKalmanFilter(const float angular_cmd[3],
  const struct KalmanCoeffiecients * k, struct KalmanState * x);

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

  // TODO: remove these temporary initializations and replace computations.
#ifdef SMALL_QUAD
  // control proportion: 0.500000
  feedback_gains_.p_dot = 3.046702738e+00;
  feedback_gains_.p = 7.144888332e+01;
  feedback_gains_.phi = 4.844637366e+02;
  feedback_gains_.phi_int = 1.085035883e+03;
  feedback_gains_.r = 5.367196113e+00;
  feedback_gains_.psi = 2.409048597e+01;
  feedback_gains_.psi_int = 2.270522950e+01;
  feedback_gains_.w_dot = -2.022580993e-01 * 9.8;
  feedback_gains_.w = 2.776627664e+00;
  feedback_gains_.z = 3.711458217e+00;
  feedback_gains_.z_int = 2.591737201e+00;
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
  attitude_model_coefficients_.p_q[0][0] = 2.744268912e-04;
  attitude_model_coefficients_.p_q[0][1] = 2.469601072e-04;
  attitude_model_coefficients_.p_q[1][0] = -1.691698564e+00;
  attitude_model_coefficients_.p_q[1][1] = 7.289510825e-01;
  attitude_model_coefficients_.r[0][0] = 1.395671668e-02;
  attitude_model_coefficients_.r[0][1] = -1.339214299e-02;
  attitude_model_coefficients_.r[1][0] = -1.851291018e+00;
  attitude_model_coefficients_.r[1][1] = 8.543211959e-01;
#else
  // control proportion: 0.400000
  feedback_gains_.p_dot = 6.125465888e-01;
  feedback_gains_.p = 1.620762355e+01;
  feedback_gains_.phi = 6.256019555e+01;
  feedback_gains_.phi_int = 7.978628452e+01;
  feedback_gains_.r = 2.493839316e+00;
  feedback_gains_.psi = 3.316200008e+00;
  feedback_gains_.psi_int = 1.453949640e+00;
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
  attitude_model_coefficients_.p_q[0][0] = 4.104693647e-04;
  attitude_model_coefficients_.p_q[0][1] = 3.865637733e-04;
  attitude_model_coefficients_.p_q[1][0] = -1.822375822e+00;
  attitude_model_coefficients_.p_q[1][1] = 8.352938353e-01;
  attitude_model_coefficients_.r[0][0] = 1.700238472e-02;
  attitude_model_coefficients_.r[0][1] = -1.619532952e-02;
  attitude_model_coefficients_.r[1][0] = -1.853281890e+00;
  attitude_model_coefficients_.r[1][1] = 8.552945558e-01;
  UPDATE ME
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

  limits_.attitude_integral = MAX_G_B_CMD * 0.5 * feedback_gains_.phi
    / feedback_gains_.phi_int;
  limits_.heading_integral = MAX_HEADING_RATE * 0.5 * feedback_gains_.psi
    / feedback_gains_.psi_int;

  limits_.altitude_error = 1.0;
  limits_.altitude_integral = 8.0;

  // TODO remove:
  thrust_cmd_to_z_integral = 1.0 / feedback_gains_.z_int
    / actuation_inverse_[0][3];
}

// -----------------------------------------------------------------------------
void Control(void)
{
  float g_b_cmd[2];  // Target x and y components of the gravity vector in body
  float heading_rate_cmd, thrust_cmd;

  // TODO: add routines to form commands from an external source.
  // Derive a target attitude from the position of the sticks.
  CommandsFromSticks(g_b_cmd, &heading_cmd_, &heading_rate_cmd);
  QuaternionFromGravityAndHeadingCommand(g_b_cmd, heading_cmd_, quat_cmd_);

  // Update the integral paths.
  UpdateIntegrals(&attitude_model_, &limits_, attitude_integral_);

  // Update the pitch and roll Kalman filters before recomputing the command.
  UpdateKalmanFilter(angular_cmd_, &kalman_coefficients_, &kalman_state_);

  // Compute a new attitude acceleration command.
  FormAngularCommand(quat_cmd_, heading_rate_cmd, &kalman_state_,
    attitude_integral_, &feedback_gains_, &limits_, angular_cmd_);

  // Compute a thrust command
  thrust_cmd = ThrustCommand(&feedback_gains_, &limits_);

  int16_t limit = FloatToS16(thrust_cmd * 2.0);
  if (limit > MAX_CMD) limit = MAX_CMD;
  for (uint8_t i = NMotors(); i--; )
    setpoints_[i] = (uint16_t)S16Limit(FloatToS16(thrust_cmd
      + Vector3Dot(angular_cmd_, actuation_inverse_[i])), MIN_CMD, limit);

  if (MotorsRunning())
    for (uint8_t i = NMotors(); i--; ) SetMotorSetpoint(i, setpoints_[i]);
  else if (MotorsStarting())
    for (uint8_t i = NMotors(); i--; ) SetMotorSetpoint(i, CONTROL_MOTORS_IDLE);
  else
    for (uint8_t i = NMotors(); i--; ) SetMotorSetpoint(i, 0);

  TxMotorSetpoints();

  // Update the model for the next time step.
  UpdateAttitudeModel(quat_cmd_, heading_rate_cmd, &feedback_gains_,
    &attitude_model_coefficients_, &attitude_model_);
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
  const float attitude_integral[3], const struct FeedbackGains * k,
  const struct Limits * limits, float angular_cmd[3])
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
    + k->phi * attitude_error[X_BODY_AXIS]
    + k->phi_int * attitude_integral[X_BODY_AXIS];
  angular_cmd[Y_BODY_AXIS] =
    + k->p_dot * -kalman->q_dot
    + k->p * (rate_cmd[Y_BODY_AXIS] - AngularRate(Y_BODY_AXIS))
    + k->phi * attitude_error[Y_BODY_AXIS]
    + k->phi_int * attitude_integral[Y_BODY_AXIS];
  angular_cmd[Z_BODY_AXIS] =
    + k->r * (rate_cmd[Z_BODY_AXIS] - AngularRate(Z_BODY_AXIS))
    + k->psi * attitude_error[Z_BODY_AXIS]
    + k->psi_int * attitude_integral[Z_BODY_AXIS];
}

// -----------------------------------------------------------------------------
static float ThrustCommand(const struct FeedbackGains * k,
  const struct Limits * limits)
{
  // TODO: make these inputs
  static int16_t hover_thrust_stick = 0;
  static float z_integral = 0.0;
  static float z_cmd = 0.0;
  static enum VerticalControlState vertical_control_state_pv = 0;

  // TODO: remove
  static float thrust_cmd_pv = 0.0;

  float thrust_cmd = 0;

  if (VerticalControlState() == VERTICAL_CONTROL_STATE_MANUAL)
  {
    const float k_sbus_to_thrust_ = (float)(MAX_THRUST_CMD - MIN_THRUST_CMD)
      / (2.0 * (float)SBUS_MAX);
    thrust_cmd = (float)(SBusThrust() + SBUS_MAX) * k_sbus_to_thrust_
      + MIN_THRUST_CMD;
  }
  else
  {
    float w, z;

    // if (VerticalControlState() == VERTICAL_CONTROL_STATE_BARO())
    // {
      if (VerticalControlState() != vertical_control_state_pv)
      {
        z_cmd = -DeltaPressureAltitude();

        // TODO: compute these
        hover_thrust_stick = SBusThrust();
        z_integral = thrust_cmd_to_z_integral * thrust_cmd_pv;
      }
      z = -DeltaPressureAltitude();
      w = -VerticalSpeed();
    // }
    // else
    // {
    //   if (VerticalControlState() != vertical_control_state_pv)
    //     z_cmd = PositionVector()[2];
    //   z = PositionVector()[2];
    //   w = VelocityVector()[2];
    // }

    float w_cmd = (float)(hover_thrust_stick - SBusThrust())
      / (float)(SBUS_MAX);
    float w_error = FloatLimit(w_cmd - w, -10.0, 10.0);
    float z_error = FloatLimit(z_cmd - z, -limits->altitude_error,
      limits->altitude_error);

    // TODO: do this actuation inverse multiplication in a smarter way!
    thrust_cmd = FloatLimit(actuation_inverse_[0][3] * (
      + k->w_dot * -(Acceleration(Z_BODY_AXIS) + 1.0)
      + k->w * w_error
      + k->z * z_error
      + k->z_int * z_integral),
      MIN_THRUST_CMD, MAX_THRUST_CMD);

    // TODO: do this elsewhere
    z_integral += z_error * DT;
    z_cmd += w_cmd * DT;
  }

  thrust_cmd_pv = thrust_cmd;

  return thrust_cmd;
}

// -----------------------------------------------------------------------------
// This function updates the attitude model that is used for the model-based
// integral action.
static void UpdateAttitudeModel(const float quat_cmd[4], float heading_rate_cmd,
  const struct FeedbackGains * k, const struct AttitudeModelCoefficients * c,
  struct AttitudeModel * model)
{
  // Compute the error between the model's attitude and the command.
  float attitude_error[3];
  AttitudeError(quat_cmd, model->quat, attitude_error);

  // Distribute the heading rate command to the axes.
  float g_b[3], rate_cmd[3];
  UpdateGravtiyInBody(model->quat, g_b);
  Vector3Scale(g_b, heading_rate_cmd, rate_cmd);

  float angular_rate[3];
  angular_rate[X_BODY_AXIS] = DirectForm2ZeroB0(k->p * rate_cmd[X_BODY_AXIS]
    + k->phi * attitude_error[X_BODY_AXIS], c->p_q, model->delays[X_BODY_AXIS]);
  angular_rate[Y_BODY_AXIS] = DirectForm2ZeroB0(k->p * rate_cmd[Y_BODY_AXIS]
    + k->phi * attitude_error[Y_BODY_AXIS], c->p_q, model->delays[Y_BODY_AXIS]);

  angular_rate[Z_BODY_AXIS] = DirectForm2ZeroB0(k->r * rate_cmd[Z_BODY_AXIS]
    + k->psi * attitude_error[Z_BODY_AXIS], c->r, model->delays[Z_BODY_AXIS]);

  UpdateQuaternion(model->quat, angular_rate, DT);
  QuaternionNormalizingFilter(model->quat);
}

// -----------------------------------------------------------------------------
// This function updates the error integrals.
static void UpdateIntegrals(const struct AttitudeModel * model,
  const struct Limits * limits, float attitude_integral[3])
{
  // Compute the error between the actual and model attitudes.
  float attitude_error[3];
  AttitudeError(model->quat, Quat(), attitude_error);

  // TODO: make a symmetric limiter
  attitude_integral[X_BODY_AXIS] = FloatLimit(attitude_integral[X_BODY_AXIS]
    + attitude_error[X_BODY_AXIS] * DT, -limits->attitude_integral,
    limits->attitude_integral);
  attitude_integral[Y_BODY_AXIS] = FloatLimit(attitude_integral[Y_BODY_AXIS]
    + attitude_error[Y_BODY_AXIS] * DT, -limits->attitude_integral,
    limits->attitude_integral);
  attitude_integral[Z_BODY_AXIS] = FloatLimit(attitude_integral[Z_BODY_AXIS]
    + attitude_error[Z_BODY_AXIS] * DT, -limits->heading_integral,
    limits->heading_integral);
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
