#include <avr/interrupt.h>

#include "adc.h"
#include "attitude.h"
#include "battery.h"
#include "buzzer.h"
#include "control.h"
#include "i2c.h"
#include "led.h"
#include "mcu_pins.h"
#include "motors.h"
#include "pressure_altitude.h"
#include "sbus.h"
#include "state.h"
#include "timing.h"
#include "uart.h"


// ============================================================================+
// Private data:

static volatile uint8_t flag_128hz_ = 0, flag_2hz_ = 0;
static volatile uint16_t main_overrun_count_ = 0;


// =============================================================================
// Private function declarations:

int16_t main(void) __attribute__ ((noreturn));


// =============================================================================
// Public functions:

void PreflightInit(void)
{
  if (!MotorsInhibited()) return;
  ZeroGyros();
  ResetPressureSensorRange();
  ResetAttitude();
  main_overrun_count_ = 0;
  RedLEDOff();
}

// -----------------------------------------------------------------------------
void SensorCalibration(void)
{
  if (!MotorsInhibited()) return;
  ZeroAccelerometers();
  PressureSensorBiasCalibration();
  ResetAttitude();
  main_overrun_count_ = 0;
  RedLEDOff();
}


// =============================================================================
// Private functions:

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

  sei();  // Allow other interrupts to be serviced.

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

// -----------------------------------------------------------------------------
static void Init(void)
{
  TimingInit();
  LEDInit();
  BuzzerInit();
  I2CInit();
  UARTInit();
  SBusInit();
  PressureSensorInit();

  // Pull up the version pin (FlightCtrl V2.2 will be grounded).
  VERSION_2_2_PORT |= VERSION_2_2_PIN | VERSION_2_2_PIN;

  sei();  // Enable interrupts

  UARTPrintf("University of Tokyo Mikrokopter firmware V2");

  LoadGyroOffsets();
  LoadAccelerometerOffsets();
  ADCOn();  // Start reading the sensors

  ResetPressureSensorRange();

  DetectBattery();
  DetectMotors();
  ControlInit();  // Must be run after DetectMotors() to get NMotors()
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Init();

  // TODO: Delete these temporary EEPROM settings.
  // SBusSetChannels(2, 3, 0, 1, 17);
  // SetNMotors(4);
  // float b_inv[8][4] = {
  //   {  3.3856,  3.3856, -46.6148, 62.2433 },
  //   { -3.3856, -3.3856, -46.6148, 62.2433 },
  //   { -3.3856,  3.3856,  46.6148, 62.2433 },
  //   {  3.3856, -3.3856,  46.6148, 62.2433 }
  // };
  // SetActuationInverse(b_inv);

  main_overrun_count_ = 0;
  RedLEDOff();

  // Main loop
  for (;;)  // Preferred over while(1)
  {
    if (flag_128hz_)
    {
      ProcessSensorReadings();

      UpdateAttitude();

      UpdateSBus();

      UpdateState();

      Control();

      flag_128hz_ = 0;
      if (main_overrun_count_) RedLEDOn();
    }

    if (flag_2hz_)
    {
      // Control();

      GreenLEDToggle();
      flag_2hz_ = 0;
    }
  }

  for (;;) continue;
}
