#include "main.h"

#include <avr/interrupt.h>

#include "adc.h"
#include "led.h"
#include "sbus.h"
#include "timing.h"
#include "uart.h"


// =============================================================================
// Private functions:

static void Init(void)
{
  TimingInit();
  LEDInit();
  UARTInit();
  SBusInit();

  // Enable interrupts.
  sei();
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Init();

  ADCOn();

  // Main loop
  int16_t timestamp = GetTimestampMillisFromNow(500);
  for (;;)  // Preferred over while(1)
  {
    if (TimestampInPast(timestamp))
    {
      timestamp += 50;
      GreenLEDToggle();
      ProcessSensorReadings();
      UARTTxByte(Acceleration(X_AXIS));
    }
  }
}
