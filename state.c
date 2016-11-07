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
#include "uart.h"
#include "vector.h"


// =============================================================================
// Private data:

static enum StateBits state_ = STATE_BIT_MOTORS_INHIBITED;
static enum ControlMode control_mode_ = CONTROL_MODE_MANUAL;

static enum ControlStateBits {
  CONTROL_STATE_BIT_NAV_MODE_0            = 1<<0,
  CONTROL_STATE_BIT_NAV_MODE_1            = 1<<1,
  CONTROL_STATE_BIT_NAV_CONTROL_INHIBITED = 1<<2,
  CONTROL_STATE_BIT_ALTITUDE_CONTROL      = 1<<3,
  CONTROL_STATE_BIT_TAKEOFF               = 1<<4,
} control_state_ = 0x00;


// =============================================================================
// Private function declarations:

static uint8_t SafetyCheck(void);
static void UpdateControlMode(void);


// =============================================================================
// Accessors

uint8_t AltitudeControlActive(void)
{
  return control_state_ & CONTROL_STATE_BIT_ALTITUDE_CONTROL;
}

// -----------------------------------------------------------------------------
enum ControlMode ControlMode(void)
{
  return control_mode_;
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

// -----------------------------------------------------------------------------
enum NavMode NavModeRequest(void)
{
  return (enum NavMode)(control_state_ & 0x03);
}

// -----------------------------------------------------------------------------
enum StateBits State(void)
{
  return state_;
}

// -----------------------------------------------------------------------------
uint8_t Takeoff(void)
{
  return control_state_ & CONTROL_STATE_BIT_TAKEOFF;
}


// =============================================================================
// Public functions:

void ClearTakeoffMode(void)
{
  control_state_ &= ~CONTROL_STATE_BIT_TAKEOFF;
}

// -----------------------------------------------------------------------------
void UpdateState(void)
{
  static uint8_t sbus_on_off_latch = 0;
  static uint16_t stick_timer;

  if (MotorsInhibited())
  {
    if (SBusThrustStickUp() && SBusYawStickLeft())
    {
      if (TimestampInPast(stick_timer))
      {
        stick_timer = GetTimestampMillisFromNow(2000);
        PreflightInit();
        if (SafetyCheck())
        {
          state_ |= STATE_BIT_INITIALIZED;
          state_ ^= STATE_BIT_INITIALIZATION_TOGGLE;
        }
        else
        {
          state_ &= ~STATE_BIT_INITIALIZED;
          BeepPattern(0x0000AAAA);
        }
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

  UpdateControlMode();
}


// =============================================================================
// Private functions:

static uint8_t SafetyCheck(void)
{
  if (MotorsRunning()) return 1;

  if (PressureAltitudeError())
  {
    UARTPrintf("state: failed pressure altitude safety check");
    return 0;
  }
  if (BLCErrorBits())
  {
    UARTPrintf("state: failed motor controller safety check");
    return 0;
  }
  if (Vector3NormSquared(AngularRateVector()) > 0.01)
  {
    UARTPrintf("state: failed gyro safety check");
    return 0;
  }
  if (fabs(Vector3NormSquared(AccelerationVector()) - 1.0) > 0.1)
  {
    UARTPrintf("state: failed accelerometer safety check");
    return 0;
  }
  if (fabs(GravityInBodyVector()[2] - 1.0) > 0.05)
  {
    UARTPrintf("state: failed level surface safety check");
    return 0;
  }
  return 1;
}

// -----------------------------------------------------------------------------
static void SetNavMode(enum NavMode mode)
{
  control_state_ &= ~0x03;  // Clear the nav mode bits
  control_state_ |= mode;
}

// -----------------------------------------------------------------------------
// This function manages the control mode, including the following modes:
// manual, barometric vertical speed, position hold, waypoint, takeoff, and come
// home. Note that position hold, waypoint, and takeoff ignore the status of
// barometric vertical speed control in favor of position estimates/commands
// from the nav. Also note that position hold and waypoint control are inhibited
// when manual intervention is detected.
static void UpdateControlMode(void)
{
  static int16_t thrust_stick_0 = 0;
  static uint8_t altitude_switch_pv = SBUS_SWITCH_CENTER;
  static uint8_t nav_switch_pv = SBUS_SWITCH_CENTER;
  static uint8_t takeoff_switch_pv = SBUS_SWITCH_CENTER;

  if (SBusNavControl() != nav_switch_pv)
  {
    switch (SBusNavControl())
    {
      case SBUS_SWITCH_DOWN:
        SetNavMode(NAV_MODE_OFF);
        break;
      case SBUS_SWITCH_CENTER:
        SetNavMode(NAV_MODE_HOLD);
        break;
      case SBUS_SWITCH_UP:
        SetNavMode(NAV_MODE_AUTO);
        break;
    }

    // Clear the inhibit latch and set the reference thrust stick position when
    // transitioning to position control.
    if (nav_switch_pv == SBUS_SWITCH_DOWN)
    {
      control_state_ &= ~CONTROL_STATE_BIT_NAV_CONTROL_INHIBITED;
      thrust_stick_0 = SBusThrust();
    }
  }

  if (SBusAltitudeControl() != altitude_switch_pv)
  {
    if (SBusAltitudeControl() == SBUS_SWITCH_UP)
    {
      control_state_ |= CONTROL_STATE_BIT_ALTITUDE_CONTROL;
      thrust_stick_0 = SBusThrust();
    }
    else
    {
      control_state_ &= ~CONTROL_STATE_BIT_ALTITUDE_CONTROL;
    }
  }

  if (SBusTakeoff() != takeoff_switch_pv)
  {
    if (SBusTakeoff() == SBUS_SWITCH_UP && SBusThrustStickDown())
    {
      control_state_ |= CONTROL_STATE_BIT_TAKEOFF
        | CONTROL_STATE_BIT_NAV_CONTROL_INHIBITED;
      control_state_ &= ~CONTROL_STATE_BIT_ALTITUDE_CONTROL;
    }
    if (SBusTakeoff() != SBUS_SWITCH_UP)
    {
      control_state_ &= ~CONTROL_STATE_BIT_TAKEOFF;
    }
  }
  // TODO: Disable takeoff engagement if thrust stick is not centered.

  // Latch the nav control inhibit bit if there is any thrust stick movement or
  // any other sticks deviate from center.
  if ((abs(SBusThrust() - thrust_stick_0) > 10) || !SBusPitchStickCentered()
    || !SBusRollStickCentered() || !SBusYawStickCentered())
  {
    control_state_ |= CONTROL_STATE_BIT_NAV_CONTROL_INHIBITED;
    if ((control_mode_ == CONTROL_MODE_TAKEOFF_TO_NAV)
      || (control_mode_ == CONTROL_MODE_TAKEOFF_TO_BARO))
    {
      control_state_ &= ~CONTROL_STATE_BIT_TAKEOFF
       & ~CONTROL_STATE_BIT_ALTITUDE_CONTROL;
    }
  }

  // Disable nav control if the inhibit bit is latched.
  if (control_state_ & CONTROL_STATE_BIT_NAV_CONTROL_INHIBITED)
  {
    control_state_ &= ~0x03;  // Clear the nav mode bits
  }

  if ((NavModeRequest() != NAV_MODE_OFF) && (NavMode() != NAV_MODE_OFF))
  {
    if (control_state_ & CONTROL_STATE_BIT_TAKEOFF)
    {
      control_mode_ = CONTROL_MODE_TAKEOFF_TO_NAV;
    }
    else
    {
      control_mode_ = CONTROL_MODE_NAV;
    }
  }
  else if (control_state_ & CONTROL_STATE_BIT_ALTITUDE_CONTROL)
  {
    if (control_state_ & CONTROL_STATE_BIT_TAKEOFF)
    {
      control_mode_ = CONTROL_MODE_TAKEOFF_TO_BARO;
    }
    else
    {
      control_mode_ = CONTROL_MODE_BARO_ALTITUDE;
    }
  }
  else if (control_state_ & CONTROL_STATE_BIT_TAKEOFF)
  {
    control_mode_ = CONTROL_MODE_PRE_TAKEOFF;
  }
  else
  {
    control_mode_ = CONTROL_MODE_MANUAL;
  }

  // Set the past values.
  altitude_switch_pv = SBusAltitudeControl();
  nav_switch_pv = SBusNavControl();
  takeoff_switch_pv = SBusTakeoff();
}
