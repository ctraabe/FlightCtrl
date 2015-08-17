#include "led.h"

#include "mcu_pins.h"


// =============================================================================
// Public functions:

void LEDInit(void)
{
  // Set onboard LED pins to output.
  LED_DDR |= GREEN_LED_PIN | RED_LED_PIN;
  // Set external LED pins to output.
  EXTERNAL_LED_DDR |= EXTERNAL_LED_1_PIN | EXTERNAL_LED_3_PIN;

  ExternalLED1Off();
  ExternalLED3Off();
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
void ExternalLED1Off(void)
{
  EXTERNAL_LED_PORT &= ~EXTERNAL_LED_1_PIN;
}

// -----------------------------------------------------------------------------
void ExternalLED1On(void)
{
  EXTERNAL_LED_PORT |= EXTERNAL_LED_1_PIN;
}

// -----------------------------------------------------------------------------
void ExternalLED1Toggle(void)
{
  EXTERNAL_LED_PORT ^= EXTERNAL_LED_1_PIN;
}

// -----------------------------------------------------------------------------
void ExternalLED3Off(void)
{
  EXTERNAL_LED_PORT &= ~EXTERNAL_LED_3_PIN;
}

// -----------------------------------------------------------------------------
void ExternalLED3On(void)
{
  EXTERNAL_LED_PORT |= EXTERNAL_LED_3_PIN;
}

// -----------------------------------------------------------------------------
void ExternalLED3Toggle(void)
{
  EXTERNAL_LED_PORT ^= EXTERNAL_LED_3_PIN;
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
