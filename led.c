#include "led.h"


// ============================================================================+
// Public functions:

void LEDInit(void)
{
  // Set onboard LED pins to output.
  LED_DDR |= GREEN_LED_PIN | RED_LED_PIN;
}

