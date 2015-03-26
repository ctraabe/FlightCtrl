#include "led.h"


// =============================================================================
// Public functions:

void LEDInit(void)
{
  // Set onboard LED pins to output.
  LED_DDR |= GREEN_LED_PIN | RED_LED_PIN;
}

// -----------------------------------------------------------------------------
void GreenLEDOff(void)
{
  LED_PORT |= GREEN_LED_PIN;
}

// -----------------------------------------------------------------------------
void GreenLEDOn(void)
{
  LED_PORT &= ~GREEN_LED_PIN;
}

// -----------------------------------------------------------------------------
void GreenLEDToggle(void)
{
  LED_PORT ^= GREEN_LED_PIN;
}

// -----------------------------------------------------------------------------
void RedLEDOff(void)
{
  LED_PORT &= ~RED_LED_PIN;
}

// -----------------------------------------------------------------------------
void RedLEDOn(void)
{
  LED_PORT |= RED_LED_PIN;
}

// -----------------------------------------------------------------------------
void RedLEDToggle(void)
{
  LED_PORT ^= RED_LED_PIN;
}
