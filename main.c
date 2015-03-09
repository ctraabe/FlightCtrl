#include "main.h"

#include <avr/interrupt.h>

#include "adc.h"
#include "led.h"
#include "pressure_altitude.h"
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
  PressureSensorInit();

  // Enable interrupts.
  sei();
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Init();

  ADCOn();

  Wait(1000);

  ResetPressureSensorRange();

  // Main loop
  int16_t timestamp = GetTimestampMillisFromNow(500);
  uint8_t message[10], flag = 0, counter = 200;
  for (;;)  // Preferred over while(1)
  {
    if (TimestampInPast(timestamp))
    {
      timestamp += 200;
      // GreenLEDToggle();
      // ProcessSensorReadings();
      uint8_t i = PrintU16(PressureSensor(), &message[0]);
      i += PrintEOL(&message[i]);
      for (uint8_t j = 0; j < i; j++)
      {
        UARTTxByte(message[j]);
      }
      if (!--counter)
      {
        counter = 200;
        if (flag) OCR0B -= 8;
        else OCR0B += 8;
        flag = !flag;
      }
    }
      ProcessSBus();
  }
}
