#include "led.h"


// =============================================================================
// Public functions:

void LEDInit(void)
{
  // Set onboard LED pins to output.
  LED_DDR |= GREEN_LED_PIN | RED_LED_PIN;
}

// -----------------------------------------------------------------------------
inline void GreenLEDOff(void)
{
  GREEN_LED_PORT |= GREEN_LED_PIN;
}

// -----------------------------------------------------------------------------
inline void GreenLEDOn(void)
{
  GREEN_LED_PORT &= ~GREEN_LED_PIN;
}

// -----------------------------------------------------------------------------
inline void GreenLEDToggle(void)
{
  GREEN_LED_PORT ^= GREEN_LED_PIN;
}

// -----------------------------------------------------------------------------
inline void RedLEDOff(void)
{
  RED_LED_PORT &= ~RED_LED_PIN;
}

// -----------------------------------------------------------------------------
inline void RedLEDOn(void)
{
  RED_LED_PORT |= RED_LED_PIN;
}

// -----------------------------------------------------------------------------
inline void RedLEDToggle(void)
{
  RED_LED_PORT ^= RED_LED_PIN;
}
