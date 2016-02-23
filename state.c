#include "state.h"

#include <stdlib.h>

#include "adc.h"
#include "attitude.h"
#include "buzzer.h"
#include "main.h"
#include "motors.h"
#include "nav_comms.h"
#include "pressure_altitude.h"
#include "sbus.h"
#include "timing.h"
#include "vector.h"
// TODO: remove
#include "led.h"


// =============================================================================
// Private data:

static enum StateBits state_ = STATE_BIT_MOTORS_INHIBITED;

static enum ControlModeBits {
  CONTROL_MODE_BIT_NAV_MODE_0            = 1<<0,
  CONTROL_MODE_BIT_NAV_MODE_1            = 1<<1,
  CONTROL_MODE_BIT_NAV_CONTROL_INHIBITED = 1<<2,
  CONTROL_MODE_BIT_ALTITUDE_CONTROL      = 1<<3,
  CONTROL_MODE_BIT_TAKEOFF               = 1<<4,
} control_mode_ = 0x00;


// =============================================================================
// Private function declarations:

static uint8_t SafetyCheck(void);
static void UpdateControlState(void);


// =============================================================================
// Accessors

uint8_t AltitudeControlActive(void)
{
  return control_mode_ & CONTROL_MODE_BIT_ALTITUDE_CONTROL;
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
  return (enum NavMode)(control_mode_ & 0x03);
}

// -----------------------------------------------------------------------------
enum StateBits State(void)
{
  return state_;
}

// -----------------------------------------------------------------------------
uint8_t Takeoff(void)
{
  return control_mode_ & CONTROL_MODE_BIT_TAKEOFF;
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
        // TODO: move the safety check to preflight to main.c
        if (SafetyCheck())
        {
          state_ |= STATE_BIT_INITIALIZED;
        }
        else
        {
          RedLEDOn();
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

  UpdateControlState();
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

// -----------------------------------------------------------------------------
static void SetNavMode(enum NavMode mode)
{
  control_mode_ &= ~0x03;  // Clear the nav mode bits
  control_mode_ |= mode;
}

// -----------------------------------------------------------------------------
static void UpdateControlState(void)
{
  static int16_t thrust_stick_0 = 0;
  static uint8_t nav_switch_pv = SBUS_SWITCH_CENTER;
  static uint8_t altitude_switch_pv = SBUS_SWITCH_CENTER;

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
      control_mode_ &= ~CONTROL_MODE_BIT_NAV_CONTROL_INHIBITED;
      thrust_stick_0 = SBusThrust();
    }

    // Arm takeoff mode by engaging automatic control with manual altitude
    // control and thrust stick down.
    // if ((SBusNavControl() != SBUS_SWITCH_DOWN) && SBusThrustStickDown()
    //     && (SBusAltitudeControl() != SBUS_SWITCH_UP))
    // {
    //   control_mode_ |= CONTROL_MODE_BIT_TAKEOFF;
    // }
  }

  if (SBusAltitudeControl() != altitude_switch_pv)
  {
    if (SBusAltitudeControl() == SBUS_SWITCH_UP)
    {
      control_mode_ |= CONTROL_MODE_BIT_ALTITUDE_CONTROL;
      thrust_stick_0 = SBusThrust();

      // Disable takeoff if engaged without the thrust stick centered.
      // if ((control_mode_ & CONTROL_MODE_BIT_TAKEOFF)
      //   && !SBusThrustStickCentered())
      // {
      //   control_mode_ &= ~CONTROL_MODE_BIT_TAKEOFF;
      // }
    }
    else
    {
      control_mode_ &= ~CONTROL_MODE_BIT_ALTITUDE_CONTROL;
    }
  }

  // Allow thrust_stick movement in takeoff mode.
  // if ((control_mode_ & CONTROL_MODE_BIT_TAKEOFF)
  //   && !(control_mode_ & CONTROL_MODE_BIT_ALTITUDE_CONTROL))
  // {
  //   thrust_stick_0 = SBusThrust();
  // }

  // Latch the nav control inhibit bit if there is any thrust stick movement or
  // any other sticks deviate from center.
  if ((abs(SBusThrust() - thrust_stick_0) > 10) || !SBusPitchStickCentered()
    || !SBusRollStickCentered() || !SBusYawStickCentered())
  {
    control_mode_ |= CONTROL_MODE_BIT_NAV_CONTROL_INHIBITED;
  }

  // Disable nav control if the inhibit bit is latched.
  if (control_mode_ & CONTROL_MODE_BIT_NAV_CONTROL_INHIBITED)
  {
    control_mode_ &= ~0x03;  // Clear the nav mode bits
  }

  // Set the past values.
  nav_switch_pv = SBusNavControl();
  altitude_switch_pv = SBusAltitudeControl();
}
