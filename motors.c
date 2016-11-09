#include "motors.h"

#include <string.h>

#include "eeprom.h"
#include "i2c.h"
#include "uart.h"


// =============================================================================
// Private data:

#define MOTORS_BASE_ADDRESS (0x52)

enum BLCStatusCode
{
  BLC_STATUS_UNKNOWN = 0,
  BLC_STATUS_MISMATCH = 1,  // Arbitrary
  BLC_STATUS_STARTING = 40,
  BLC_STATUS_AFRO_ESC = 239,  // AfroESC with custom UT firmware
  BLC_STATUS_BLC_TO_PPM = 240,  // Custom BLCtrl to PPM translator board
  BLC_STATUS_V3_FAST_READY = 248,
  BLC_STATUS_V3_READY = 249,
  BLC_STATUS_V2_READY = 250,
  BLC_STATUS_RUNNING_REDUNDANT = 254,
  BLC_STATUS_RUNNING = 255,  // V1 always gives this before motors are started
};

enum BLCFeatureBits
{
  BLC_FEATURE_EXTENDED_COMMS = 1<<0,
  BLC_FEATURE_V3 = 1<<1,
  BLC_FEATURE_20KHz = 1<<2,
};

enum BLCConfigBits
{
  BLC_CONFIG_BIT_REVERSE_ROTATION = 1<<0,
  BLC_CONFIG_BIT_START_PWM_1 = 1<<1,
  BLC_CONFIG_BIT_START_PWM_2 = 1<<2,
  BLC_CONFIG_BIT_START_PWM_3 = 1<<3,
};

struct MotorSetpoint
{
  uint8_t bits_11_to_3;
  uint8_t bits_2_to_0;
} __attribute__((packed));

struct BLCConfig
{
  uint8_t revision;  // BLC configuration revision
  uint8_t mask;  // Settings mask
  uint8_t pwm_scaling;  // PWM saturation
  uint8_t current_limit;  // Current limit in A
  uint8_t temperature_limit;  // °C
  uint8_t current_scaling;  // Scale factor for current measurement
  uint8_t bit_field;
  uint8_t checksum;
} __attribute__((packed));

struct BLCStatus
{
  uint8_t current;  // x 0.1 A
  enum BLCStatusCode status_code;  // Also command limit when running?
  uint8_t temperature;  // °C (for V2 or greater, 0xFF otherwise)
  uint8_t speed;  // rad/s / 5.79
  uint8_t extra;  // V3: Voltage, V2: mAh, V1: N/A
  uint8_t voltage;  // x 0.1V (V3 is limited to 255, V2 is only low-byte)
  uint8_t i2c_errors;  // V2 or greater
  uint8_t version_major;  // V2 or greater
  uint8_t version_minor;  // V2 or greater
} __attribute__((packed));

static uint8_t blc_error_bits_ = 0x00;
static uint8_t blc_feature_bits_ = 0x00;
static uint8_t n_motors_ = 0;
static uint8_t setpoint_length_ = sizeof(uint8_t);
static uint8_t comms_in_progress_;  // Address to which communication is ongoing

static struct MotorSetpoint setpoints_[MAX_MOTORS] = { { 0 } };
static volatile struct BLCStatus blc_status_[MAX_MOTORS] = { { 0 } };


// =============================================================================
// Private function declarations:

static void TxMotorSetpoint(uint8_t address);


// =============================================================================
// Accessors

uint8_t BLCErrorBits(void)
{
  return blc_error_bits_;
}

// -----------------------------------------------------------------------------
uint8_t MotorSpeed(uint8_t i)
{
  return blc_status_[i].speed;
}

// -----------------------------------------------------------------------------
uint8_t NMotors(void)
{
  return n_motors_;
}


// =============================================================================
// Public functions:

// This function pings all of the possible brushless motor controller addresses
// by sending a 0 command. A response indicates that a controller (and hopefully
// also a motor) is present. The contents of the response indicate the type and
// features of the controller.
void DetectMotors(void)
{
  // TODO: if (motors_on) return;

  // Send a 0 command to each brushless controller address and record any
  // responses.
  uint8_t motors = 0;  // Bit field representing motors present.
  uint8_t setpoint = 0;  // Do not command the motors to move
  enum BLCStatusCode blc_status_code = BLC_STATUS_UNKNOWN;
  for (uint8_t i = 0; i < MAX_MOTORS; i++)
  {
    I2CTxThenRx(MOTORS_BASE_ADDRESS + (i << 1), &setpoint, sizeof(setpoint),
      (volatile uint8_t *)&blc_status_[i], sizeof(struct BLCStatus));
    I2CWaitUntilCompletion();
    // I2C will give an error if there is no response.
    if (!I2CError())
    {
      motors |= (1 << i);  // Mark this motor as present

      // Check that all controllers are the same type.
      if (blc_status_code == BLC_STATUS_UNKNOWN)
        blc_status_code = blc_status_[i].status_code;
      else if (blc_status_[i].status_code != blc_status_code)
        blc_error_bits_ |= BLC_ERROR_BIT_INCONSISTENT_SETTINGS;
    }
  }

  // Check for missing or extra motors. Assumes that present motors have
  // contiguous addresses beginning with 0.
  n_motors_ = eeprom_read_byte(&eeprom.n_motors);
  if (((1 << n_motors_) - 1) & !motors)
    blc_error_bits_ |= BLC_ERROR_BIT_MISSING_MOTOR;
  if (motors & !((1 << n_motors_) - 1))
    blc_error_bits_ |= BLC_ERROR_BIT_EXTRA_MOTOR;
  if (blc_error_bits_ & ~_BV(BLC_ERROR_BIT_INCONSISTENT_SETTINGS))
  {
    UARTPrintf("motors: ERROR: expected controllers with addresses: 0 - %i",
      n_motors_ - 1);
    UARTPrintf("  Bit field of responding addresses is: %X", motors);
    return;
  }

  // Identify additional features of the brushless controllers.
  UARTPrintf("motors: detected controllers with the following compatibility:");
  switch (blc_status_code)
  {
    case BLC_STATUS_V3_FAST_READY:
      blc_feature_bits_ |= BLC_FEATURE_20KHz;
    case BLC_STATUS_V3_READY:
      blc_feature_bits_ |= BLC_FEATURE_V3;
    case BLC_STATUS_V2_READY:
      blc_feature_bits_ |= BLC_FEATURE_EXTENDED_COMMS;
    case BLC_STATUS_AFRO_ESC:
    case BLC_STATUS_BLC_TO_PPM:
      setpoint_length_ = sizeof(uint16_t);
    default:
      break;
  }

  // Report successful detection.
  if (blc_status_code == BLC_STATUS_V2_READY)
    UARTPrintf("motors: detected %u V2 controllers", n_motors_);
  else if (blc_status_code == BLC_STATUS_V3_READY)
    UARTPrintf("motors: detected %u V3 controllers", n_motors_);
  else if (blc_status_code == BLC_STATUS_V3_FAST_READY)
    UARTPrintf("motors: detected %u V3 controllers in fast mode (20 kHz PWM)",
      n_motors_);
  else
    UARTPrintf("motors: detected %u V1 controllers", n_motors_);
}

// -----------------------------------------------------------------------------
uint8_t MotorsStarting(void)
{
  uint8_t result = 0;
  for (uint8_t i = n_motors_; i--; )
    result |= blc_status_[i].status_code == BLC_STATUS_STARTING;
  return result;
}

// -----------------------------------------------------------------------------
void SetMotorSetpoint(uint8_t address, uint16_t setpoint)
{
  if (address >= MAX_MOTORS) return;
  setpoints_[address].bits_2_to_0 = (uint8_t)setpoint & 0x7;
  setpoints_[address].bits_11_to_3 = (uint8_t)(setpoint >> 3);
}

// -----------------------------------------------------------------------------
void SetNMotors(uint8_t n_motors)
{
  if (n_motors > MAX_MOTORS) n_motors = MAX_MOTORS;
  eeprom_update_byte(&eeprom.n_motors, n_motors);
  DetectMotors();
}

// -----------------------------------------------------------------------------
void TxMotorSetpoints(void)
{
  comms_in_progress_ = n_motors_ - 1;
  TxMotorSetpoint(comms_in_progress_);
}


// =============================================================================
// Private functions:

static void TxNextMotorSetpoint(void)
{
  if (comms_in_progress_--) TxMotorSetpoint(comms_in_progress_);
}

// -----------------------------------------------------------------------------
static void TxMotorSetpoint(uint8_t address)
{
  I2CTxThenRxThenCallback(MOTORS_BASE_ADDRESS + (address << 1),
    (uint8_t *)&setpoints_[address], setpoint_length_,
    (volatile uint8_t *)&blc_status_[address], sizeof(struct BLCStatus),
    TxNextMotorSetpoint);
}
