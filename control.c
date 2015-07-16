#include "control.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>

#include "adc.h"
#include "attitude.h"
#include "eeprom.h"
#include "main.h"
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

static float actuation_inverse_[MOTORS_MAX][4];
static float attitude_error_limit_, heading_error_limit_;
static float k_phi_, k_p_, k_psi_, k_r_;
static float k_sbus_to_thrust_, min_thrust_cmd_, max_thrust_cmd_;


// =============================================================================
// Private function declarations:

static float * AttitudeCommand(float g_b_cmd[2], float * heading_cmd,
  float * heading_rate_cmd, float attitude_cmd[3]);
static void CommandsFromSticks(float g_b_cmd[2], float * heading_cmd,
  float * heading_rate_cmd, float * thrust_cmd);


// =============================================================================
// Public functions:

void ControlInit(void)
{
  eeprom_read_block((void*)actuation_inverse_,
    (const void*)&eeprom.actuation_inverse[0][0], sizeof(actuation_inverse_));

  // TODO: remove these temporary initializations and replace with EEPROM.
  k_phi_ = 100.0;
  k_p_ = 28.0;
  // k_p_dot_ = 2.15;
  k_psi_ = 7.0711;
  k_r_ = 4.3439;

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
  CommandsFromSticks(g_b_cmd, &heading_cmd, &heading_rate_cmd, &thrust_cmd);

  float attitude_cmd[3];
  AttitudeCommand(g_b_cmd, &heading_cmd, &heading_rate_cmd, attitude_cmd);
/*
  for (uint8_t i = NMotors(); i--; )
  {
    setpoint[i] = U16Limit(thrust_cmd, MIN_CMD - CMD_MARGIN, MAX_CMD
      + CMD_MARGIN);
  }
*/
  if (MotorsRunning())
  {
    for (uint8_t i = NMotors(); i--; )
    {
      SetMotorSetpoint(i, (uint16_t)S16Limit(FloatToS16(thrust_cmd
        * actuation_inverse_[i][3] + VectorDot(attitude_cmd,
        actuation_inverse_[i])), MIN_CMD, MAX_CMD));
    }
  }
  else if (MotorsStarting())
  {
    for (uint8_t i = NMotors(); i--; )
      SetMotorSetpoint(i, CONTROL_MOTORS_IDLE);
  }
  else
  {
    for (uint8_t i = NMotors(); i--; )
      SetMotorSetpoint(i, 0);
  }

  TxMotorSetpoints();
}

// -----------------------------------------------------------------------------
void SetActuationInverse(float actuation_inverse[MOTORS_MAX][4])
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
    + k_p_ * (rate_cmd[X_BODY_AXIS] - AngularRate(X_BODY_AXIS));
  attitude_cmd[Y_BODY_AXIS] = k_phi_ * attitude_error[Y_BODY_AXIS]
    + k_p_ * (rate_cmd[Y_BODY_AXIS] - AngularRate(Y_BODY_AXIS));
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
