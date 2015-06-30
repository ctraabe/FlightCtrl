#include "control.h"

#include <math.h>
#include <stdlib.h>

#include "attitude.h"
#include "eeprom.h"
#include "main.h"
#include "motors.h"
#include "sbus.h"
#include "state.h"
#include "uart.h"
#include "vector.h"


// =============================================================================
// Private data:

#define CONTROL_MOTORS_IDLE (40)
#define MAX_G_B_CMD (sin(M_PI / 3.0))

static float b_inverse_[MOTORS_MAX][4];


// =============================================================================
// Private function declarations:

static float * AttitudeFromSticks(float g_b_command[3]);


// =============================================================================
// Public functions:

void ControlInit(void)
{
  eeprom_read_block((void*)b_inverse_, (const void*)&eeprom.b_inverse[0][0],
    sizeof(b_inverse_));
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
  uint16_t setpoint[MOTORS_MAX];
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

  float attitude_command[3];
  AttitudeFromSticks(attitude_command);

  float attitude_error[3];
  VectorCross(attitude_command, GravityInBodyVector(), attitude_error);

  float x_b_command = 100.0 * attitude_error[X_BODY_AXIS];
  float y_b_command = 100.0 * attitude_error[Y_BODY_AXIS];

  UARTPrintf("%f %f", x_b_command, y_b_command);

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
static float * AttitudeFromSticks(float g_b_command[3])
{
  float x = (float)SBusPitch() * MAX_G_B_CMD / (float)SBUS_MAX;
  g_b_command[X_BODY_AXIS] = x - x * (float)abs(SBusRoll()) * MAX_G_B_CMD
    / (4.0 * (float)SBUS_MAX);

  float y = (float)SBusRoll() * MAX_G_B_CMD / (float)SBUS_MAX;
  g_b_command[Y_BODY_AXIS] = -y + y * (float)abs(SBusPitch()) * MAX_G_B_CMD
    / (4.0 * (float)SBUS_MAX);

  g_b_command[Z_BODY_AXIS] = sqrt(1.0 - square(g_b_command[X_BODY_AXIS])
    - square(g_b_command[Y_BODY_AXIS]));

  return g_b_command;
}
