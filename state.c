#include "state.h"

#include "adc.h"
#include "attitude.h"
#include "buzzer.h"
#include "main.h"
#include "motors.h"
#include "pressure_altitude.h"
#include "sbus.h"
#include "timing.h"
#include "vector.h"


// =============================================================================
// Private data:

enum ExternalControlBits {
  EXTERNAL_CONTROL_BIT_INHIBITED = 1<<0,
  EXTERNAL_CONTROL_BIT_ACTIVE = 1<<1,
};

static enum StateBits state_ = STATE_BIT_MOTORS_INHIBITED;
static enum ExternalControlBits external_control_ = 0;


// =============================================================================
// Private function declarations:

static uint8_t SafetyCheck(void);


// =============================================================================
// Accessors

enum StateBits State(void)
{
  return state_;
}

// -----------------------------------------------------------------------------
uint8_t MotorsInhibited(void)
{
  return state_ & STATE_BIT_MOTORS_INHIBITED;
}

// -----------------------------------------------------------------------------
uint8_t MotorsRunning(void)
{
  return state_ & STATE_BIT_MOTORS_RUNNING;
}


// =============================================================================
// Public functions:

void UpdateState(void)
{
  static uint8_t sbus_on_off_latch = 0;
  static uint16_t stick_timer;

  if (state_ & STATE_BIT_MOTORS_INHIBITED)
  {
    if (SBusThrustStickUp() && SBusYawStickLeft())
    {
      if (TimestampInPast(stick_timer))
      {
        stick_timer = GetTimestampMillisFromNow(2000);
        PreflightInit();
        if (SafetyCheck()) state_ |= STATE_BIT_INITIALIZED;
      }
    }
    else if (SBusThrustStickUp() && SBusYawStickRight())
    {
      if (TimestampInPast(stick_timer))
      {
        stick_timer = GetTimestampMillisFromNow(2000);
        SensorCalibration();
      }
    }
    else if (SBusOnOff() && SBusThrustStickDown() && !sbus_on_off_latch)
    {
      if (TimestampInPast(stick_timer))
      {
        if (state_ & STATE_BIT_INITIALIZED)
        {
          state_ &= ~STATE_BIT_MOTORS_INHIBITED;
          state_ |= STATE_BIT_MOTORS_RUNNING;
        }
        else
        {
          BeepPattern(0x0000AAAA);
        }
        sbus_on_off_latch = 1;
      }
    }
    else
    {
      stick_timer = GetTimestampMillisFromNow(1000);
    }
  }
  else
  {
    if (SBusOnOff() && !sbus_on_off_latch && SBusThrustStickDown())
    {
      if (TimestampInPast(stick_timer))
      {
        state_ |= STATE_BIT_MOTORS_INHIBITED;
        state_ &= ~STATE_BIT_MOTORS_RUNNING;
        sbus_on_off_latch = 1;
      }
    }
    else
    {
      stick_timer = GetTimestampMillisFromNow(250);
    }
  }

  if (sbus_on_off_latch && !SBusOnOff()) sbus_on_off_latch = 0;
}

// -----------------------------------------------------------------------------
void ExternalControlMode(void)
{
  static int16_t thrust_stick_0 = 0;
  int16_t ec_switch = SBusSwitch(external_control_channel);

  // Latch the inhibit bit if there is any thrust stick movement.
  if (ec_switch >= 0)
  {
    external_control_ |= -(abs(SBusThrust() - thrust_stick_0) > 10)
      & EXTERNAL_CONTROL_BIT_INHIBITED;
  }
  else
  {
    external_control_ &= ~EXTERNAL_CONTROL_BIT_INHIBITED;
  }

  // External control request bit
  if ((ec_switch > 0) && SBusThrustStickDown() && !(external_control_
    & EXTERNAL_CONTROL_BIT_INHIBITED))
  {
    external_control_ |= EXTERNAL_CONTROL_BIT_ACTIVE;
  }
  else
  {
    external_control_ &= ~EXTERNAL_CONTROL_BIT_ACTIVE;
    thrust_stick_initial = SBusThrust();
  }

  // Takeoff mode request bit (latches to allow moving thrust stick to center)
  if (to_switch >= 0 && (((thrust_stick < 30) && (ec_switch < 0))
      || (MKToPTAM.request & PTAMToMK.status & PTAM_TAKEOFF))) {
    MKToPTAM.request |= PTAM_TAKEOFF;
  } else {
    MKToPTAM.request &= ~PTAM_TAKEOFF;
  }
}


// =============================================================================
// Private functions:

static uint8_t SafetyCheck(void)
{
  if (PressureAltitudeError()) return 0;
  if (BLCErrorBits()) return 0;
  if (Vector3NormSquared(AngularRateVector()) > 0.01) return 0;
  if (Vector3NormSquared(AccelerationVector()) > 1.1) return 0;
  if (Vector3NormSquared(AccelerationVector()) < 0.9) return 0;
  if (GravityInBodyVector()[2] < 0.9) return 0;
  return 1;
}
