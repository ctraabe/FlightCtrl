#include "control.h"

#include "motors.h"
#include "state.h"


// =============================================================================
// Private data:

#define CONTROL_MOTORS_IDLE (40)


// =============================================================================
// Public functions:

void Control(void)
{
  if (MotorsRunning())
  {
    SetMotorSetpoint(1, CONTROL_MOTORS_IDLE - 10);
  }
  else if (MotorsStarting())
  {
    SetMotorSetpoint(1, CONTROL_MOTORS_IDLE);
  }
  else
  {
    SetMotorSetpoint(1, 0);
  }
}
