#include "main.h"

#include <avr/interrupt.h>

#include "mcu_pins.h"
#include "timing.h"


void Init(void)
{
  // Set onboard LED pins to output.
  LED_GREEN_DDR |= LED_GREEN_PIN;
  LED_RED_DDR |= LED_RED_PIN;

  // Initialize TIMER3 for millisecond timing.
  TimingInit();

  // Enable interrupts.
  sei();
}

int16_t main(void)
{
  Init();

  // Main loop
  uint16_t timestamp = GetTimestampMillisFromNow(500);
  for (;;)  // Preferred over while(1)
  {
    if(TimestampInPast(timestamp))
    {
      timestamp += 500;
      LED_GREEN_TOGGLE;
    }
  }
}
