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

static enum StateBits state_ = STATE_BIT_MOTORS_INHIBITED;


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
