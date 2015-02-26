#include "main.h"

#include <avr/interrupt.h>

#include "led.h"
#include "timing.h"
#include "uart.h"


// ============================================================================+
// Private functions:

static void Init(void)
{
  TimingInit();
  LEDInit();
  UARTInit();

  // Enable interrupts.
  sei();
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Init();

  // Main loop
  int16_t timestamp = GetTimestampMillisFromNow(500);
  for (;;)  // Preferred over while(1)
  {
    if(TimestampInPast(timestamp))
    {
      timestamp += 500;
      GreenLEDToggle();
    }
  }
}
