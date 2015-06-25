#include <avr/interrupt.h>

#include "adc.h"
#include "attitude.h"
#include "battery.h"
#include "buzzer.h"
#include "i2c.h"
#include "led.h"
#include "motors.h"
#include "pressure_altitude.h"
#include "sbus.h"
#include "timing.h"
#include "uart.h"


// ============================================================================+
// Private data:

static volatile uint8_t flag_128hz = 0, flag_2hz = 0;
static volatile uint16_t main_overrun_count = 0;


// =============================================================================
// Private function declarations:

int16_t main(void) __attribute__ ((noreturn));


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
      flag_2hz = 1;
    case COUNTER_4HZ:
    case COUNTER_8HZ:
    case COUNTER_16HZ:
      UpdateBuzzer();
    case COUNTER_32HZ:
    case COUNTER_64HZ:
    case COUNTER_128HZ:
      if (flag_128hz) main_overrun_count++;
      else flag_128hz = 1;
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

  sei();  // Enable interrupts

  UARTPrintf("University of Tokyo Mikrokopter firmware V2");

  LoadAccelerometerOffsets();
  ADCOn();  // Start reading the sensors

  ResetPressureSensorRange();

  DetectBattery();
  DetectMotors();
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Init();

  ZeroGyros();
  // ZeroAccelerometers();

  // Main loop
  for (;;)  // Preferred over while(1)
  {
    if (flag_128hz)
    {
      ProcessSensorReadings();

      UpdateAttitude();

      ProcessSBus();
/*
      SetMotorSetpoint(0, (uint16_t)S16Limit(SBusChannel(0), 0, 800));
      SetMotorSetpoint(1, (uint16_t)S16Limit(SBusChannel(1), 0, 800));
      SetMotorSetpoint(2, (uint16_t)S16Limit(SBusChannel(2), 0, 800));
      SetMotorSetpoint(3, (uint16_t)S16Limit(SBusChannel(3), 0, 800));
      TxMotorSetpoints();
*/
      flag_128hz = 0;
    }

    if (flag_2hz)
    {
      // UARTPrintf("SBus: %i, %i, %i, %i", SBusChannel(0), SBusChannel(1), SBusChannel(2), SBusChannel(3));
      // UARTPrintf("%f %f %f %f", Quat(0), Quat(1), Quat(2), Quat(3));
      // UARTPrintf("%f %f %f", Gravity(0), Gravity(1), Gravity(2));
      // UARTPrintf("%f %f %f", AngularRate(0), AngularRate(1), AngularRate(2));
      UARTPrintf("%f %f %f", Acceleration(0), Acceleration(1), Acceleration(2));
      // UARTPrintf("%f", QuatCorrection());
      // UARTPrintf("%f", HeadingAngle());

      GreenLEDToggle();
      flag_2hz = 0;
    }
  }

  // for (;;) continue;
}
