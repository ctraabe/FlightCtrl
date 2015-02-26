#ifndef LED_H_
#define LED_H_

#include "mcu_pins.h"


// ============================================================================+
// Inline functions:

inline void GreenLEDOff(void)
{
  GREEN_LED_PORT &= ~GREEN_LED_PIN;
}

// -----------------------------------------------------------------------------
inline void GreenLEDOn(void)
{
  GREEN_LED_PORT |= GREEN_LED_PIN;
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


// ============================================================================+
// Public functions:

void LEDInit(void);


#endif // LED_H_
