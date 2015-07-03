#include "control.h"

#include <math.h>
#include <stdlib.h>

#include "adc.h"
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
static float p_dot_to_cmd_;
static float k_phi_, k_p_;
static float attitude_error_limit_;
static int16_t  max_cmd_from_attitude_error_;
static uint16_t k_sbus_to_thrust_, min_cmd_from_thrust_, max_cmd_from_thrust_;


// =============================================================================
// Private function declarations:

static float * AttitudeFromSticks(float g_b_cmd[3]);
static int16_t * AttitudeCommand(int16_t attitude_cmd[3]);
static uint16_t ThrustCommand(void);


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
  uint16_t cmd_from_thrust = ThrustCommand();

  int16_t cmd_from_attitude[3];
  AttitudeCommand(cmd_from_attitude);
/*
  for (uint8_t i = NMotors(); i--; )
  {
    setpoint[i] = U16Limit(cmd_from_thrust, MIN_CMD - CMD_MARGIN, MAX_CMD
      + CMD_MARGIN);
  }
*/
  if (MotorsRunning())
  {
    for (uint8_t i = 1; i--; )
    {
      setpoint[i] = cmd_from_thrust + 0 * cmd_from_attitude[0];
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

// This function computes an attitude command in the form of desired components
// of the gravity vector along the x and y body axes. WARNING: THE NORM OF THESE
// COMPONENTS SHOULD NEVER EXCEED ONE!!!
static float * AttitudeFromSticks(float g_b_cmd[3])
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
static int16_t * AttitudeCommand(int16_t attitude_cmd[3])
{
  float attitude_cmd_f[3];
  AttitudeFromSticks(attitude_cmd_f);

  // The following computes a vector along the axis of rotation from the gravity
  // vector in the body axis to the desired gravity vector. The magnitude of the
  // vector is equal to 2 * sin(phi / 2), where phi is the angle between the
  // current and desired gravity vectors.
  float attitude_error[3];
  VectorGain(VectorCross(attitude_cmd_f, GravityInBodyVector(), attitude_error),
    1.0 / sqrt(0.5 + 0.5 * VectorDot(attitude_cmd_f, GravityInBodyVector())),
    attitude_error);

  // Saturate the error.
  float attitude_error_norm = VectorNorm(attitude_error);
  if (attitude_error_norm > attitude_error_limit_)
    VectorGain(attitude_error, attitude_error_limit_
      / attitude_error_norm, attitude_error);

  VectorGain(attitude_error, k_phi_, attitude_cmd_f);
  VectorGainAndAccumulate(AngularRateVector(), k_p_, attitude_cmd_f);
  VectorGain(attitude_cmd_f, p_dot_to_cmd_, attitude_cmd_f);

  // TODO: fix this so that negative numbers round correctly
  attitude_cmd[0] = (int16_t)(attitude_cmd_f[0] + 0.5);
  attitude_cmd[1] = (int16_t)(attitude_cmd_f[1] + 0.5);
  attitude_cmd[2] = (int16_t)(attitude_cmd_f[2] + 0.5);

  return attitude_cmd;
}

// -----------------------------------------------------------------------------
// This function uses some fixed-point math tricks to efficiently compute the
// thrust contribution from a pre-computed thrust multiplier (Q9).
static uint16_t ThrustCommand(void)
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
