#include "main.h"

#include <avr/interrupt.h>

#include "adc.h"
#include "led.h"
#include "print.h"
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
  uint8_t message[10];
  for (;;)  // Preferred over while(1)
  {
    if (TimestampInPast(timestamp))
    {
      timestamp += 50;
      GreenLEDToggle();
      ProcessSensorReadings();
      uint8_t i = PrintS16(SBusChannel(0), &message[0]);
      i += PrintEOL(&message[i]);
      for (uint8_t j = 0; j < i; j++)
      {
        UARTTxByte(message[j]);
      }
    }
      ProcessSBus();
  }
}
