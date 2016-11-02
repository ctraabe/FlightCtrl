#include <avr/interrupt.h>

#include "adc.h"
#include "attitude.h"
#include "battery.h"
#include "buzzer.h"
#include "control.h"
#include "i2c.h"
#include "indicator.h"
#include "led.h"
#include "mcu_pins.h"
#include "mk_serial_protocol.h"
#include "mk_serial_tx.h"
#include "motors.h"
#include "nav_comms.h"
#include "pressure_altitude.h"
#include "sbus.h"
#include "spi.h"
#include "state.h"
#include "timing.h"
#include "uart.h"
#include "vertical_speed.h"

#ifdef MOTOR_TEST
  #include "motor_test.h"
#endif


// ============================================================================+
// Private data:

static volatile uint8_t flag_128hz_ = 0, flag_64hz_ = 0, flag_2hz_ = 0;
static volatile uint16_t main_overrun_count_ = 0;
static uint8_t board_version = 0;


// =============================================================================
// Private function declarations:

void ResetOverrun(void);


// =============================================================================
// Accessors:

uint8_t BoardVersion(void)
{
  return board_version;
}


// =============================================================================
// Public functions:

void PreflightInit(void)
{
  if (!MotorsInhibited()) return;
  BeepDuration(100);
  ZeroGyros();
  ResetPressureSensorRange();
  ResetAttitude();
  BeepDuration(500);
  WaitForBuzzerToComplete();
  // Warn if any automatic flight modes are armed.
  if ((SBusAltitudeControl() != SBUS_SWITCH_DOWN)
    || (SBusNavControl() != SBUS_SWITCH_DOWN))
  {
    BeepPattern(0x2AA);
    WaitForBuzzerToComplete();
  }
  ResetOverrun();
}

// -----------------------------------------------------------------------------
void SensorCalibration(void)
{
  if (!MotorsInhibited()) return;
  BeepDuration(100);
  ZeroAccelerometers();
  ResetAttitude();
  BeepDuration(500);
  WaitForBuzzerToComplete();
  ResetOverrun();
}


// =============================================================================
// Private functions:

static void Init(void)
{
  // Check the board version.
  if (PINB & 0x02)
  {
    // Pull up the version pin (FlightCtrl V2.2 will be grounded).
    VERSION_2_2_PORT |= VERSION_2_2_PIN;
    if (VERSION_2_2)
      board_version = 22;
    else
      board_version = 21;
  }
  else
  {
    board_version = 25;
  }

  LEDInit();
  RedLEDOn();

  TimingInit();
  BuzzerInit();
  PressureSensorInit();
  I2CInit();
  UARTInit();
  SPIInit();
  SBusInit();

  sei();  // Enable interrupts

  UARTPrintf("\n\rUniversity of Tokyo FlightCtrl firmware V2\n\r");
  UARTPrintf("MikroKopter FlightCtrl version %i detected", board_version);

  LoadGyroOffsets();
  LoadAccelerometerOffsets();
  ADCOn();  // Start reading the sensors

  ResetPressureSensorRange();

  DetectBattery();
  DetectMotors();
  ControlInit();  // Must be run after DetectMotors() to get NMotors()

  NavCommsInit();
  IndicatorInit();

  ResetOverrun();
  GreenLEDOn();
}

// -----------------------------------------------------------------------------
void ErrorCheck(void)
{
  // Order from lowest to highest priority.
  if (main_overrun_count_ > 10) BeepPattern(0x000000AA);

  if (BatteryLow()) BeepPattern(0x000000CC);

  static uint8_t sbus_stale_pv = 0;
  if (SBusStale() && (!MotorsInhibited() || !sbus_stale_pv))
    BeepPattern(0x0CFCFCFC);  // dit dah dah dah
  if (sbus_stale_pv && !SBusStale())
    BeepDuration(500);

  sbus_stale_pv = SBusStale();
}

// -----------------------------------------------------------------------------
void ResetOverrun(void)
{
  flag_128hz_ = 0;
  main_overrun_count_ = 0;
  RedLEDOff();
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Init();

#ifdef MOTOR_TEST
  MotorTest();
  ResetOverrun();
#endif
/*
  // TODO: Delete these temporary EEPROM settings.
  SBusSetChannels(1, 0, 3, 2, 17, 16, 5, 7, 4, 6, 6, 6, 6, 6, 8, 9, 10, 11);
#if defined BI_OCTO
  SetNMotors(8);
  float b_inv[8][4] = {
    { +0.000000000e+00, +9.675320382e+00, +1.972471902e+02, -6.033975571e+01 },
    { +9.308934042e+00, +6.841484653e+00, -1.972471902e+02, -6.033975571e+01 },
    { +1.316482077e+01, +0.000000000e+00, +1.972471902e+02, -6.033975571e+01 },
    { +9.308934042e+00, -6.841484653e+00, -1.972471902e+02, -6.033975571e+01 },
    { +0.000000000e+00, -9.675320382e+00, +1.972471902e+02, -6.033975571e+01 },
    { -9.308934042e+00, -6.841484653e+00, -1.972471902e+02, -6.033975571e+01 },
    { -1.316482077e+01, +0.000000000e+00, +1.972471902e+02, -6.033975571e+01 },
    { -9.308934042e+00, +6.841484653e+00, -1.972471902e+02, -6.033975571e+01 },
  };
  SetActuationInverse(b_inv);
#elif defined SMALL_QUAD
  SetNMotors(4);
  float b_inv[8][4] = {
    { +2.416975886e+00, +2.416975886e+00, -4.971845548e+01, -5.047879731e+01 },
    { -2.416975886e+00, -2.416975886e+00, -4.971845548e+01, -5.047879731e+01 },
    { -2.416975886e+00, +2.416975886e+00, +4.971845548e+01, -5.047879731e+01 },
    { +2.416975886e+00, -2.416975886e+00, +4.971845548e+01, -5.047879731e+01 },
  };
  SetActuationInverse(b_inv);
#else  // Large quad
  SetNMotors(4);
  float b_inv[8][4] = {
    { +0.000000000e+00, +5.820423590e+00, -5.469661607e+01, -5.576508516e+01 },
    { +0.000000000e+00, -5.820423590e+00, -5.469661607e+01, -5.576508516e+01 },
    { -5.847173231e+00, +0.000000000e+00, +5.469661607e+01, -5.576508516e+01 },
    { +5.847173231e+00, +0.000000000e+00, +5.469661607e+01, -5.576508516e+01 },
  };
  SetActuationInverse(b_inv);
#endif
*/
  // Main loop
  for (;;)  // Preferred over while(1)
  {
    if (flag_128hz_)
    {

      UpdateSBus();
      UpdateState();

      ProcessSensorReadings();

      UpdateAttitude();
      UpdatePressureAltitude();
      UpdateVerticalSpeed();

      Control();

      ErrorCheck();

      NotifyNav();

      ProcessIncomingUART();
      SendPendingUART();

      if (main_overrun_count_) RedLEDOn();

      flag_128hz_ = 0;
    }

    if (NavDataReady()) ExchangeDataWithNav();

    if (NavRecieved()) ProcessDataFromNav();

    if (flag_64hz_)
    {
      flag_64hz_ = 0;
    }

    if (flag_2hz_)
    {
      flag_2hz_ = 0;
    }
  }
}

// -----------------------------------------------------------------------------
// This function is called upon the interrupt that occurs when TIMER3 reaches
// the value in ICR3. This should occur at a rate of 128 Hz.
ISR(TIMER3_CAPT_vect)
{
  enum {
    COUNTER_128HZ = 0xFF >> 7,
    COUNTER_64HZ = 0xFF >> 6,
    COUNTER_32HZ = 0xFF >> 5,
    COUNTER_16HZ = 0xFF >> 4,
    COUNTER_8HZ = 0xFF >> 3,
    COUNTER_4HZ = 0xFF >> 2,
    COUNTER_2HZ = 0xFF >> 1,
    COUNTER_1HZ = 0xFF >> 0,
  };

  sei();  // Allow other interrupts to be serviced

  static uint8_t counter = 0;
  switch ((uint8_t)(counter ^ (counter + 1))) {
    case COUNTER_1HZ:
    case COUNTER_2HZ:
      flag_2hz_ = 1;
    case COUNTER_4HZ:
    case COUNTER_8HZ:
    case COUNTER_16HZ:
      UpdateBuzzer();
    case COUNTER_32HZ:
    case COUNTER_64HZ:
      flag_64hz_ = 1;
    case COUNTER_128HZ:
      if (flag_128hz_)
        main_overrun_count_++;
      else
        flag_128hz_ = 1;
    default:
      counter++;
      break;
  }
}
