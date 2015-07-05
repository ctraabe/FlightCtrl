#include "control.h"

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
#define MAX_HEADING_RATE (0.5)

static float b_inverse_[MOTORS_MAX][4];
static float p_dot_to_cmd_;
static float k_phi_, k_p_;
static float attitude_error_limit_;
static int16_t  max_cmd_from_attitude_error_;
static uint16_t k_sbus_to_thrust_, min_cmd_from_thrust_, max_cmd_from_thrust_;


// =============================================================================
// Private function declarations:

static int16_t * AttitudeCommand(float g_b_cmd[2], float * heading_cmd,
  int16_t attitude_cmd[3]);
static void AttitudeFromSticks(float g_b_cmd[2], float * heading_cmd);
static uint16_t ThrustCommandFromStick(uint16_t * thrust_cmd);


// =============================================================================
// Public functions:

void ControlInit(void)
{
  eeprom_read_block((void*)b_inverse_, (const void*)&eeprom.b_inverse[0][0],
    sizeof(b_inverse_));

  // TODO: remove these temporary initializations and replace with EEPROM.
  float omega_2_to_cmd = 0.003236649;
  float p_dot_to_omega_2 = 1538.461538462;
  p_dot_to_cmd_ = omega_2_to_cmd * p_dot_to_omega_2;
  k_phi_ = 100.0;
  k_p_ = 30.0;
  // k_p_dot_ = 2.15;

  // Compute limits on the attitude that will give the specified approach speed
  // when error is very large.
  attitude_error_limit_ = MAX_ATTITUDE_RATE * k_p_ / k_phi_;
  max_cmd_from_attitude_error_ = (int16_t)(k_phi_ * attitude_error_limit_
    * p_dot_to_cmd_ + 0.5);

  // Compute the thrust command range based on the margin that is required for
  // attitude control. Thrust is computed directly from the thrust stick using
  // fixed-point math. The Q9 stick gain is precomputed below.
  min_cmd_from_thrust_ = MIN_CMD + max_cmd_from_attitude_error_;
  max_cmd_from_thrust_ = MAX_CMD - max_cmd_from_attitude_error_;
  k_sbus_to_thrust_ = (uint16_t)(((float)max_cmd_from_thrust_
    - (float)min_cmd_from_thrust_) / (2.0 * (float)SBUS_MAX) * (float)(1 << 9));
}

// -----------------------------------------------------------------------------
void Control(void)
{
  uint16_t setpoint[MOTORS_MAX];
  uint16_t thrust_cmd;
  float g_b_cmd[2];  // Target x and y components of the gravity vector in body
  static float heading_cmd;

  // TODO: add routines to form commands from an external source.
  AttitudeFromSticks(g_b_cmd, &heading_cmd);
  ThrustCommandFromStick(&thrust_cmd);

  int16_t attitude_cmd[3];
  AttitudeCommand(g_b_cmd, &heading_cmd, attitude_cmd);
/*
  for (uint8_t i = NMotors(); i--; )
  {
    setpoint[i] = U16Limit(thrust_cmd, MIN_CMD - CMD_MARGIN, MAX_CMD
      + CMD_MARGIN);
  }
*/
  if (MotorsRunning())
  {
    for (uint8_t i = 1; i--; )
    {
      setpoint[i] = thrust_cmd + 0 * attitude_cmd[0];
      SetMotorSetpoint(i, setpoint[i]);
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
void SetBInverse(float b_inverse[MOTORS_MAX][4])
{
  eeprom_update_block((const void*)b_inverse, (void*)&eeprom.b_inverse[0][0],
    sizeof(b_inverse_));
  ControlInit();
}


// =============================================================================
// Private functions:

static int16_t * AttitudeCommand(float g_b_cmd[2], float * heading_cmd,
  int16_t attitude_cmd[3])
{
  // Compute the z component of the gravity vector command.
  float g_b_cmd_z = sqrt(1.0 - square(g_b_cmd[X_BODY_AXIS])
    - square(g_b_cmd[Y_BODY_AXIS]));

  // Form a quaternion from these components (z component is 0).
  float temp1 = 0.5 + 0.5 * square(g_b_cmd_z);
  float quat_g_b_cmd_0 = sqrt(temp1);
  float temp2 = 1.0 / (2.0 * quat_g_b_cmd_0);
  float quat_g_b_cmd_x = g_b_cmd[Y_BODY_AXIS] * temp2;
  float quat_g_b_cmd_y = -g_b_cmd[X_BODY_AXIS] * temp2;

  // Determine the (approximate) heading of this command for removal (optional).
  float psi_from_g_b_cmd = (quat_g_b_cmd_x * quat_g_b_cmd_y) / (temp1
    + square(quat_g_b_cmd_x) - 0.5);

  // Make a second quaternion for the commanded heading minus the residual
  // heading from the gravity vector command (x and y components are zero).
  temp1 = (*heading_cmd - psi_from_g_b_cmd) * 0.5;
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
  quat_cmd[3] = quat_g_b_cmd_0;

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

  // Apply the control gains.
  float attitude_cmd_f[3];
  VectorGain(attitude_error, k_phi_, attitude_cmd_f);
  VectorGainAndAccumulate(AngularRateVector(), k_p_, attitude_cmd_f);
  VectorGain(attitude_cmd_f, p_dot_to_cmd_, attitude_cmd_f);
  attitude_cmd[0] = FloatToS16(attitude_cmd_f[0]);
  attitude_cmd[1] = FloatToS16(attitude_cmd_f[1]);
  attitude_cmd[2] = FloatToS16(attitude_cmd_f[2]);

  return attitude_cmd;
}

// -----------------------------------------------------------------------------
void AttitudeFromSticks(float g_b_cmd[2], float * heading_cmd)
{
  g_b_cmd[X_BODY_AXIS] = (float)SBusPitch() * MAX_G_B_CMD / (float)SBUS_MAX;
  g_b_cmd[Y_BODY_AXIS] = -(float)SBusRoll() * MAX_G_B_CMD / (float)SBUS_MAX;

  // Scale back the commanded components at the corners (optional).
  g_b_cmd[X_BODY_AXIS] -= g_b_cmd[X_BODY_AXIS] * (float)abs(SBusRoll())
    * MAX_G_B_CMD / (4.0 * (float)SBUS_MAX);
  g_b_cmd[Y_BODY_AXIS] -= g_b_cmd[Y_BODY_AXIS] * (float)abs(SBusPitch())
    * MAX_G_B_CMD / (4.0 * (float)SBUS_MAX);

  float psi_dot_cmd = (float)SBusYaw() * MAX_HEADING_RATE / (float)SBUS_MAX;
  *heading_cmd += psi_dot_cmd * DT;
  while (*heading_cmd > M_PI) *heading_cmd -= 2.0 * M_PI;
  while (*heading_cmd < -M_PI) *heading_cmd += 2.0 * M_PI;
}

// -----------------------------------------------------------------------------
// This function uses some fixed-point math tricks to efficiently compute the
// thrust contribution from a pre-computed thrust multiplier (Q9).
static uint16_t ThrustCommandFromStick(uint16_t * thrust_cmd)
{
  union {
    uint32_t uint32;
    struct
    {
      uint8_t low8;
      uint16_t uint16;
      uint8_t space;
    } result;
    uint8_t bytes[4];
  } u;

  u.uint32 = (uint32_t)(SBusThrust() + SBUS_MAX) * k_sbus_to_thrust_;

  *thrust_cmd = U16RoundRShiftU16(u.result.uint16, 1) + min_cmd_from_thrust_;
  return *thrust_cmd;
}
