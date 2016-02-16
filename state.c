#include "state.h"

#include <stdlib.h>

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
static enum HorizontalControlState horizontal_control_state_ = 0;
static enum VerticalControlState vertical_control_state_ = 0;



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
void UpdateControlState(void)
{
  static int16_t thrust_stick_0 = 0;
  static uint8_t hc_switch_pv = 0, vc_switch_pv = 0;

  if (SBusHorizontalControl() != hc_switch_pv)
  {
    switch (SBusHorizontalControl())
    {
      case SBUS_SWITCH_DOWN:
        horizontal_control_state_ = HORIZONTAL_CONTROL_STATE_MANUAL;
        break;
      case SBUS_SWITCH_CENTER:
        horizontal_control_state_ = HORIZONTAL_CONTROL_STATE_HOLD;
        break;
      case SBUS_SWITCH_UP:
        horizontal_control_state_ = HORIZONTAL_CONTROL_STATE_AUTO;
        break;
    }

    if (hc_switch_pv == SBUS_SWITCH_DOWN) thrust_stick_0 = SBusThrust();

    if ((SBusHorizontalControl() != SBUS_SWITCH_DOWN) && SBusThrustStickDown()
        && (SBusVerticalControl() == SBUS_SWITCH_DOWN))
    {
      horizontal_control_state_ = HORIZONTAL_CONTROL_STATE_TAKEOFF;
    }
  }

  if (SBusVerticalControl() != vc_switch_pv)
  {
    switch (SBusVerticalControl())
    {
      case SBUS_SWITCH_DOWN:
        vertical_control_state_ = VERTICAL_CONTROL_STATE_MANUAL;
        break;
      case SBUS_SWITCH_CENTER:
        vertical_control_state_ = VERTICAL_CONTROL_STATE_BARO;
        break;
      case SBUS_SWITCH_UP:
        vertical_control_state_ = VERTICAL_CONTROL_STATE_AUTO;
        break;
    }

    // Disable takeoff if engaged without the thrust stick centered.
    if ((horizontal_control_state_ == HORIZONTAL_CONTROL_STATE_TAKEOFF)
      && !SBusThrustStickCentered())
    {
      state_ &= STATE_BIT_POSITION_CONTROL_INHIBITED;
    }
  }

  // Allow thrust_stick movement in takeoff mode.
  if (horizontal_control_state_ == HORIZONTAL_CONTROL_STATE_TAKEOFF)
  {
    thrust_stick_0 = SBusThrust();
  }

  // Latch the position control inhibit bit if there is any thrust stick
  // movement or any other sticks deviate from center.
  if ((abs(SBusThrust() - thrust_stick_0) > 10) || !SBusPitchStickCentered()
    || !SBusRollStickCentered() || !SBusYawStickCentered())
  {
    state_ |= STATE_BIT_POSITION_CONTROL_INHIBITED;
  }

  // Clear the position control inhibit bit only if the horizontal control
  // switch is in manual and the vertical control switch is not in auto.
  if ((SBusHorizontalControl() == SBUS_SWITCH_DOWN) && (SBusVerticalControl()
    != SBUS_SWITCH_UP))
  {
    state_ &= ~STATE_BIT_POSITION_CONTROL_INHIBITED;
  }

  // Fall back to safer modes if position control is inhibited.
  if (state_ & STATE_BIT_POSITION_CONTROL_INHIBITED)
  {
    horizontal_control_state_ = HORIZONTAL_CONTROL_STATE_MANUAL;
    if (vertical_control_state_ == VERTICAL_CONTROL_STATE_AUTO)
      vertical_control_state_ = VERTICAL_CONTROL_STATE_BARO;
  }

  // Set the past values.
  hc_switch_pv = SBusHorizontalControl();
  vc_switch_pv = SBusVerticalControl();
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
