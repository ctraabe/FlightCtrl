#include "main.h"

#include <avr/interrupt.h>

#include "adc.h"
#include "buzzer.h"
#include "i2c.h"
#include "led.h"
#include "motors.h"
#include "pressure_altitude.h"
#include "print.h"
#include "sbus.h"
#include "timing.h"
#include "uart.h"


// ============================================================================+
// Private data:

static volatile uint8_t flag_128hz = 0, flag_2Hz = 0;
static volatile uint16_t main_overrun_count = 0;


// =============================================================================
// Private functions:

static void Init(void)
{
  TimingInit();
  LEDInit();
  I2CInit();
  UARTInit();
  SBusInit();
  PressureSensorInit();

  LoadAccelerometerOffsets();

  sei();  // Enable interrupts

  ADCOn();  // Start reading the sensors

  ResetPressureSensorRange();
  DetectMotors();
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

  sei();  // Allow other interrupts to be serviced.

  static uint8_t counter = 0;
  switch ((uint8_t)(counter ^ (counter + 1))) {
    case COUNTER_1HZ:
    case COUNTER_2HZ:
      flag_2Hz = 1;
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
int16_t main(void)
{
  Init();

  // Main loop
  for (;;)  // Preferred over while(1)
  {
    if (flag_128hz)
    {
      ProcessSensorReadings();

      // uint8_t message[10];
      // uint8_t i = PrintU16(BiasedPressure(), &message[0]);
      // i += PrintEOL(&message[i]);
      // for (uint8_t j = 0; j < i; j++) UARTTxByte(message[j]);

      ProcessSBus();
      TxMotorSetpoints();

      flag_128hz = 0;
    }

    if (flag_2Hz)
    {
      GreenLEDToggle();
      flag_2Hz = 0;
    }
  }

  // for (;;) continue;
}
