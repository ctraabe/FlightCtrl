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

  LoadAccelerometerOffsets();

  sei();  // Enable interrupts

  ADCOn();  // Start reading the sensors

  ResetPressureSensorRange();
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Init();

  // Main loop
  int16_t timestamp = GetTimestampMillisFromNow(500);
  uint8_t message[10], flag = 0, counter = 200;
  for (;;)  // Preferred over while(1)
  {
    if (TimestampInPast(timestamp))
    {
      timestamp += 200;
      // ProcessSensorReadings();
      uint8_t i = PrintU16(BiasedPressureSensor(), &message[0]);
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
