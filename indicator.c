#include "indicator.h"

#include "i2c.h"
#include "nav_comms.h"
#include "sbus.h"
#include "state.h"
#include "timing.h"


#define PCA9685_ADDRESS (0xBE)
#define PCA9685_RGB_B_REGISTER (0x2F)
#define PCA9685_RGB_G_REGISTER (0x27)
#define PCA9685_RGB_R_REGISTER (0x2B)
#define PCA9685_R1_REGISTER (0x13)
#define PCA9685_R2_REGISTER (0x17)
#define PCA9685_R3_REGISTER (0x1B)
#define PCA9685_B_REGISTER (0x1F)
#define PCA9685_G_REGISTER (0x23)


// =============================================================================
// Private data:

uint8_t tx_buffer_[2] = { 0 };


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
  // Set the PCA9685 to auto-increment the registers.
  tx_buffer_[0] = 0x00;
  tx_buffer_[1] = 0x20;
  I2CTx(PCA9685_ADDRESS, tx_buffer_, 2);
  I2CWaitUntilCompletion(100);

  // Initialize all LED registers to the off state.
  uint8_t tx_buffer[42] = { 0 };
  tx_buffer[0] = 0x09;
  tx_buffer[19-8] = 0x10;
  tx_buffer[23-8] = 0x10;
  tx_buffer[27-8] = 0x10;
  tx_buffer[31-8] = 0x10;
  tx_buffer[35-8] = 0x10;
  I2CTx(PCA9685_ADDRESS, tx_buffer, 42);
  I2CWaitUntilCompletion(100);

  LEDOn(PCA9685_R1_REGISTER);
  I2CWaitUntilCompletion(100);
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
  } PCA9685_led = GREEN;

  uint8_t rgb = 0x00;
  uint16_t blink_pattern = 0xA820;

  if (ControlMode() == CONTROL_MODE_NAV)
  {
    if (NavStatusOK())
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

  switch (PCA9685_led)
  {
    case GREEN:  // Heartbeat
      if (TimestampInPast(heartbeat_timer))
      {
        ToggleGreenLED();
        heartbeat_timer += 500;
      }
      PCA9685_led = BLUE;
      break;

    case BLUE:  // Receiving valid Nav data
      if (NavStatusOK())
        LEDOn(PCA9685_B_REGISTER);
      else
        LEDOff(PCA9685_B_REGISTER);
      PCA9685_led = RED1;
      break;

    case RED1:  // Route 0-2
      PCA9685_led = RED2;
      LEDOn(PCA9685_R1_REGISTER);
      break;

    case RED2:  // Route 1-2
      if (SBusSwitch(0) != 0)
        LEDOn(PCA9685_R2_REGISTER);
      else
        LEDOff(PCA9685_R2_REGISTER);
      PCA9685_led = RED3;
      break;

    case RED3:  // Route 2
      if (SBusSwitch(0) == 2)
        LEDOn(PCA9685_R3_REGISTER);
      else
        LEDOff(PCA9685_R3_REGISTER);
      PCA9685_led = RGB_RED;
      break;

    case RGB_RED:
      if ((rgb & RGB_R) && (blink_pattern & blink_mask))
        RGBLEDOn(PCA9685_RGB_R_REGISTER);
      else
        RGBLEDOff(PCA9685_RGB_R_REGISTER);
      PCA9685_led = RGB_GREEN;
      break;

    case RGB_GREEN:
      if ((rgb & RGB_G) && (blink_pattern & blink_mask))
        RGBLEDOn(PCA9685_RGB_G_REGISTER);
      else
        RGBLEDOff(PCA9685_RGB_G_REGISTER);
      PCA9685_led = RGB_BLUE;
      break;

    case RGB_BLUE:
      if ((rgb & RGB_B) && (blink_pattern & blink_mask))
        RGBLEDOn(PCA9685_RGB_B_REGISTER);
      else
        RGBLEDOff(PCA9685_RGB_B_REGISTER);
      blink_mask >>= 1;
      if (blink_mask == 0) blink_mask = 1 << 15;
    default:
      PCA9685_led = GREEN;
      break;
  }
}


// =============================================================================
// Private functions:

static void LEDOff(uint8_t led_register)
{
  tx_buffer_[0] = led_register;
  tx_buffer_[1] = 0x10;
  I2CTx(PCA9685_ADDRESS, tx_buffer_, 2);
}

// -----------------------------------------------------------------------------
static void LEDOn(uint8_t led_register)
{
  tx_buffer_[0] = led_register;
  tx_buffer_[1] = 0x00;
  I2CTx(PCA9685_ADDRESS, tx_buffer_, 2);
}

// -----------------------------------------------------------------------------
static void RGBLEDOff(uint8_t led_register)
{
  tx_buffer_[0] = led_register;
  tx_buffer_[1] = 0x00;
  I2CTx(PCA9685_ADDRESS, tx_buffer_, 2);
}

// -----------------------------------------------------------------------------
static void RGBLEDOn(uint8_t led_register)
{
  tx_buffer_[0] = led_register;
  tx_buffer_[1] = 0x10;
  I2CTx(PCA9685_ADDRESS, tx_buffer_, 2);
}

// -----------------------------------------------------------------------------
static void ToggleGreenLED(void)
{
  static uint8_t state = 0;
  if (state)
    LEDOff(PCA9685_G_REGISTER);
  else
    LEDOn(PCA9685_G_REGISTER);
  state = !state;
}
