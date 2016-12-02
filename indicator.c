#include "indicator.h"

#include "i2c.h"
#include "nav_comms.h"
#include "sbus.h"
#include "state.h"
#include "timing.h"

#define INDICATOR_ADDRESS (0xBE)
#define INDICATOR_RGB_B_REGISTER (0x07)
#define INDICATOR_RGB_G_REGISTER (0x0B)
#define INDICATOR_RGB_R_REGISTER (0x0F)
#define INDICATOR_R1_REGISTER (0x13)
#define INDICATOR_R2_REGISTER (0x17)
#define INDICATOR_R3_REGISTER (0x1B)
#define INDICATOR_B_REGISTER (0x1F)
#define INDICATOR_G_REGISTER (0x23)


// =============================================================================
// Private function declarations:

static void LEDOff(uint8_t led_register);
static void LEDOn(uint8_t led_register);
static void RGBLEDOff(uint8_t led_register);
static void RGBLEDOn(uint8_t led_register);
static void ToggleGreenLED(void);

// =============================================================================
// Public functions:

void IndicatorInit(void)
{
  uint8_t tx_buffer[30] = { 0 };
  tx_buffer[1] = 0x20;
  I2CTx(INDICATOR_ADDRESS, tx_buffer, 2);
  I2CWaitUntilCompletion();
  tx_buffer[0] = 0x09;
  tx_buffer[1] = 0x00;
  tx_buffer[19-8] = 0x10;
  tx_buffer[23-8] = 0x10;
  tx_buffer[27-8] = 0x10;
  tx_buffer[31-8] = 0x10;
  tx_buffer[35-8] = 0x10;
  I2CTx(INDICATOR_ADDRESS, tx_buffer, 30);
  I2CWaitUntilCompletion();

  LEDOn(INDICATOR_R1_REGISTER);
}

// -----------------------------------------------------------------------------
void UpdateIndicator(void)
{
  enum RGBColorBits {
    RGB_R = 1<<0,
    RGB_G = 1<<1,
    RGB_B = 1<<2,
  };

  static uint16_t heartbeat_timer = 0, blink_mask = 0x8000;
  static enum LED {
    GREEN,
    BLUE,
    RED1,
    RED2,
    RED3,
    RGB_RED,
    RGB_GREEN,
    RGB_BLUE,
  } indicator_led = GREEN;

  uint8_t rgb = 0x00;
  uint16_t blink_pattern = 0xA820;

  if (ControlMode() == CONTROL_MODE_NAV)
  {
    if (!NavStale() && NavStatus() == 1)
    {
      rgb = RGB_B;
      blink_pattern = 0xA820;
    }
    else
    {
      rgb = RGB_R;
      blink_pattern = 0xAAAA;
    }
  }
  else if ((SBusNavControl() != SBUS_SWITCH_DOWN)
    || SBusAltitudeControl() == SBUS_SWITCH_UP)
  {
    rgb = RGB_R | RGB_G;
    blink_pattern  = 0x9090;
  }
  else if (MotorsRunning())
  {
    rgb = RGB_G;
    blink_pattern = 0x8080;
  }
  else
  {
    rgb = 0;
  }

  switch (indicator_led)
  {
    case GREEN:  // Heartbeat
      if (TimestampInPast(heartbeat_timer))
      {
        ToggleGreenLED();
        heartbeat_timer += 500;
      }
      indicator_led = BLUE;
      break;

    case BLUE:  // Receiving valid Nav data
      if (!NavStale() && NavStatus() == 1)
        LEDOn(INDICATOR_B_REGISTER);
      else
        LEDOff(INDICATOR_B_REGISTER);
      indicator_led = RED1;
      break;

    case RED1:  // Route 0-2
      indicator_led = RED2;
      LEDOn(INDICATOR_R1_REGISTER);
      break;

    case RED2:  // Route 1-2
      if (SBusSwitch(0) != 0)
        LEDOn(INDICATOR_R2_REGISTER);
      else
        LEDOff(INDICATOR_R2_REGISTER);
      indicator_led = RED3;
      break;

    case RED3:  // Route 2
      if (SBusSwitch(0) == 2)
        LEDOn(INDICATOR_R3_REGISTER);
      else
        LEDOff(INDICATOR_R3_REGISTER);
      indicator_led = RGB_RED;
      break;

    case RGB_RED:
      if ((rgb & RGB_R) && (blink_pattern & blink_mask))
        RGBLEDOn(INDICATOR_RGB_R_REGISTER);
      else
        RGBLEDOff(INDICATOR_RGB_R_REGISTER);
      indicator_led = RGB_GREEN;
      break;

    case RGB_GREEN:
      if ((rgb & RGB_G) && (blink_pattern & blink_mask))
        RGBLEDOn(INDICATOR_RGB_G_REGISTER);
      else
        RGBLEDOff(INDICATOR_RGB_G_REGISTER);
      indicator_led = RGB_BLUE;
      break;

    case RGB_BLUE:
      if ((rgb & RGB_B) && (blink_pattern & blink_mask))
        RGBLEDOn(INDICATOR_RGB_B_REGISTER);
      else
        RGBLEDOff(INDICATOR_RGB_B_REGISTER);
      blink_mask >>= 1;
      if (blink_mask == 0) blink_mask = 1 << 15;
    default:
      indicator_led = GREEN;
      break;
  }
}


// =============================================================================
// Private functions:

static void LEDOff(uint8_t led_register)
{
  uint8_t tx_buffer[2];
  tx_buffer[0] = led_register;
  tx_buffer[1] = 0x10;
  I2CTx(INDICATOR_ADDRESS, tx_buffer, 2);
}

// -----------------------------------------------------------------------------
static void LEDOn(uint8_t led_register)
{
  uint8_t tx_buffer[2];
  tx_buffer[0] = led_register;
  tx_buffer[1] = 0x00;
  I2CTx(INDICATOR_ADDRESS, tx_buffer, 2);
}

// -----------------------------------------------------------------------------
static void RGBLEDOff(uint8_t led_register)
{
  uint8_t tx_buffer[2];
  tx_buffer[0] = led_register;
  tx_buffer[1] = 0x00;
  I2CTx(INDICATOR_ADDRESS, tx_buffer, 2);
}

// -----------------------------------------------------------------------------
static void RGBLEDOn(uint8_t led_register)
{
  uint8_t tx_buffer[2];
  tx_buffer[0] = led_register;
  tx_buffer[1] = 0x10;
  I2CTx(INDICATOR_ADDRESS, tx_buffer, 2);
}

// -----------------------------------------------------------------------------
static void ToggleGreenLED(void)
{
  static uint8_t state = 0;
  if (state)
    LEDOff(INDICATOR_G_REGISTER);
  else
    LEDOn(INDICATOR_G_REGISTER);
  state = !state;
}
