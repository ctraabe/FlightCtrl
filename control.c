#include "control.h"

#include <math.h>
#include <stdlib.h>

#include "attitude.h"
#include "eeprom.h"
#include "main.h"
#include "motors.h"
#include "mymath.h"
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

static float b_inverse_[MOTORS_MAX][4];
static float omega_2_to_cmd_, p_dot_to_omega_2_;
static float k_phi_, k_p_;
static float attitude_error_limit_;
static int16_t  max_cmd_from_attitude_error_;
static uint16_t k_sbus_to_thrust_, min_cmd_from_thrust_, max_cmd_from_thrust_;


// =============================================================================
// Private function declarations:

// static float * AttitudeFromSticks(float g_b_cmd[3]);


// =============================================================================
// Public functions:

void ControlInit(void)
{
  eeprom_read_block((void*)b_inverse_, (const void*)&eeprom.b_inverse[0][0],
    sizeof(b_inverse_));

  // TODO: remove these temporary initializations and replace with EEPROM.
  omega_2_to_cmd_ = 0.003236649;
  p_dot_to_omega_2_ = 1538.461538462;
  k_phi_ = 100.0;
  k_p_ = 30.0;
  // k_p_dot_ = 2.15;

  // Compute limits on the attitude that will give the specified approach speed
  // when error is very large.
  attitude_error_limit_ = MAX_ATTITUDE_RATE * k_p_ / k_phi_;
  max_cmd_from_attitude_error_ = (int16_t)(k_phi_ * attitude_error_limit_
    * p_dot_to_omega_2_ * omega_2_to_cmd_ + 0.5);

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
  // enum { THRUST_STICK_GAIN = 100 };  // 0.1953125 Q9
  // thrust_cmd = THRUST_STICK_GAIN * thrust_stick + 5 * (1<<9);  // Q9 N [5, 55]

  // phi_cmd = -(float)roll_stick / 128.;  // rad [-1, 1]
  // phi_err = phi_cmd - phi;  // rad

  // theta_cmd = -(float)pitch_stick / 128.;  // rad [-1, 1]
  // theta_err = theta_cmd - theta;  // rad

  // yaw_cmd = -(float)yaw_stick * 4. / 53. - 9.1320755 * r;

  // u[0] = -2.15 * dp - 30. * p + 100. * phi_err;
  // u[1] = -2.15 * dq - 30. * q + 100. * theta_err;
  // u[2] = yaw_cmd;
  // u[3] = (float)thrust_cmd / 512.;

  // int16_t omega_cmd[MOTORS_MAX];
  uint16_t setpoint[MOTORS_MAX] = { 0 };
  // for (uint8_t i = NMotors(); i--; )
  // {
  //   float omega2_cmd = 0.;
  //   for (uint8_t j = 0; j < 4; j++)
  //     omega2_cmd += kBInverse[i][j] * u[j];
  //   if (omega2_cmd > 0.)
  //     omega_cmd[i] = (uint16_t)sqrt(omega2_cmd);
  //   else
  //     omega_cmd[i] = 0;
  //   if (omega_cmd[i] > 140)
  //     setpoint[i] = U16Limit(((omega_cmd[i] - 140) * 8) / 3, 64, 1840);
  //   else
  //     setpoint[i] = 64;
  // }

/*
  float attitude_cmd[3];
  AttitudeFromSticks(attitude_cmd);

  float attitude_error[3];
  VectorCross(attitude_cmd, GravityInBodyVector(), attitude_error);

  float x_b_cmd = 100.0 * attitude_error[X_BODY_AXIS];
  float y_b_cmd = 100.0 * attitude_error[Y_BODY_AXIS];

  UARTPrintf("%f %f", x_b_cmd, y_b_cmd);
*/
  if (MotorsRunning())
  {
    for (uint8_t i = 1; i--; )
      SetMotorSetpoint(i, setpoint[i]);
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

// This function computes an attitude command in the form of desired components
// of the gravity vector along the x and y body axes. WARNING: THE NORM OF THESE
// COMPONENTS SHOULD NEVER EXCEED ONE!!!
// static float * AttitudeFromSticks(float g_b_cmd[3])
float * AttitudeFromSticks(float g_b_cmd[3])
{
  float x = (float)SBusPitch() * MAX_G_B_CMD / (float)SBUS_MAX;
  g_b_cmd[X_BODY_AXIS] = x - x * (float)abs(SBusRoll()) * MAX_G_B_CMD
    / (4.0 * (float)SBUS_MAX);

  float y = (float)SBusRoll() * MAX_G_B_CMD / (float)SBUS_MAX;
  g_b_cmd[Y_BODY_AXIS] = -y + y * (float)abs(SBusPitch()) * MAX_G_B_CMD
    / (4.0 * (float)SBUS_MAX);

  g_b_cmd[Z_BODY_AXIS] = sqrt(1.0 - square(g_b_cmd[X_BODY_AXIS])
    - square(g_b_cmd[Y_BODY_AXIS]));

  return g_b_cmd;
}

// -----------------------------------------------------------------------------
// This function uses some fixed-point math tricks to efficiently compute the
// thrust contribution from a pre-computed thrust multiplier (Q9).
uint16_t ThrustFromStick(void)
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

  return U16RoundRShiftU16(u.result.uint16, 1) + min_cmd_from_thrust_;
}
