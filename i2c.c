#include "i2c.h"

#include <avr/interrupt.h>
#include <util/twi.h>

#include "mcu_pins.h"
#include "timing.h"


// =============================================================================
// Private data:

#define F_SCL (200000L)

enum I2CMode {
  I2C_MODE_IDLE = 0,
  I2C_MODE_TX,
  I2C_MODE_RX,
  I2C_MODE_TX_THEN_RX
};

static volatile enum I2CMode i2c_mode_ = I2C_MODE_IDLE;
static volatile enum I2CError i2c_error_ = I2C_ERROR_NONE;
static volatile uint8_t rx_destination_len_ = 0, tx_source_len_ = 0;
static volatile uint8_t * volatile rx_destination_ptr_ = 0;
static const uint8_t * volatile tx_source_ptr_ = 0;

static uint8_t slave_address_ = 0x00;
static I2CCallback callback_ptr_ = 0;


// =============================================================================
// Private function declarations:

static void I2CStart(enum I2CMode i2c_mode);
static void I2CStop(void);


// =============================================================================
// Accessors:

enum I2CError I2CError(void)
{
  return i2c_error_;
}


// =============================================================================
// Public functions:

// This initialization sets the I2C pin states and clock. It should be performed
// prior to enabling interrupts.
void I2CInit(void)
{
  I2C_DDR |= I2C_SCL_PIN;  // Set SCL pin to output (necessary?)
  I2C_PORT |= I2C_SCL_PIN | I2C_SDA_PIN;  // Enable pull-ups
  TWBR = ((F_CPU / F_SCL) - 16) / 2;  // Set the bitrate.
}

// -----------------------------------------------------------------------------
uint8_t I2CIsIdle(void)
{
  return i2c_mode_ == I2C_MODE_IDLE;
}

// -----------------------------------------------------------------------------
void I2CReset(void)
{
  I2CStop();
}

// -----------------------------------------------------------------------------
enum I2CError I2CRxThenCallback(uint8_t slave_address,
  volatile uint8_t *rx_destination_ptr, uint8_t rx_destination_len,
  I2CCallback callback_ptr)
{
  if (i2c_mode_ != I2C_MODE_IDLE) return I2C_ERROR_BUSY;
  slave_address_ = slave_address;
  rx_destination_ptr_ = rx_destination_ptr;
  rx_destination_len_ = rx_destination_len;
  callback_ptr_ = callback_ptr;
  i2c_error_ = I2C_ERROR_NONE;
  I2CStart(I2C_MODE_RX);
  return I2C_ERROR_NONE;
}

// -----------------------------------------------------------------------------
enum I2CError I2CTx(uint8_t slave_address, const uint8_t *tx_source_ptr,
  uint8_t tx_source_len)
{
  return I2CTxThenRxThenCallback(slave_address, tx_source_ptr, tx_source_len,
    0, 0, (I2CCallback)0);
}

// -----------------------------------------------------------------------------
enum I2CError I2CTxThenRx(uint8_t slave_address, const uint8_t *tx_source_ptr,
  uint8_t tx_source_len, volatile uint8_t *rx_destination_ptr,
  uint8_t rx_destination_len)
{
  return I2CTxThenRxThenCallback(slave_address, tx_source_ptr, tx_source_len,
    rx_destination_ptr, rx_destination_len, (I2CCallback)0);
}

// -----------------------------------------------------------------------------
enum I2CError I2CTxThenRxThenCallback(uint8_t slave_address,
  const uint8_t *tx_source_ptr, uint8_t tx_source_len,
  volatile uint8_t *rx_destination_ptr, uint8_t rx_destination_len,
  I2CCallback callback_ptr)
{
  if (i2c_mode_ != I2C_MODE_IDLE) return I2C_ERROR_BUSY;
  slave_address_ = slave_address;
  tx_source_ptr_ = tx_source_ptr;
  tx_source_len_ = tx_source_len;
  rx_destination_ptr_ = rx_destination_ptr;
  rx_destination_len_ = rx_destination_len;
  callback_ptr_ = callback_ptr;
  i2c_error_ = I2C_ERROR_NONE;
  I2CStart(I2C_MODE_TX_THEN_RX);
  return I2C_ERROR_NONE;
}

// -----------------------------------------------------------------------------
uint8_t I2CWaitUntilCompletion(uint16_t time_limit_ms)
{
  uint16_t timeout = GetTimestampMillisFromNow(time_limit_ms);
  while ((i2c_mode_ != I2C_MODE_IDLE) && !TimestampInPast(timeout)) continue;
  if (!TimestampInPast(timeout)) return 0;

  // Reset I2C if the communication didn't complete within the expected time.
  I2CReset();
  return 1;
}


// =============================================================================
// Private functions:

static void I2CReadByte(void)
{
  *rx_destination_ptr_ = TWDR;
  rx_destination_ptr_++;
  rx_destination_len_--;
}

// -----------------------------------------------------------------------------
// Initiate data reception and acknowledge the result.
static void I2CRxAck(void)
{
  TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);
}

// -----------------------------------------------------------------------------
// Initiate data reception and do not acknowledge the result (don't send more).
static void I2CRxNAck(void)
{
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
}

// -----------------------------------------------------------------------------
// Give a start or repeated start signal.
static void I2CStart(enum I2CMode i2c_mode)
{
  i2c_mode_ = i2c_mode;
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);
}

// -----------------------------------------------------------------------------
// Give the stop signal.
static void I2CStop(void)
{
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
  i2c_mode_ = I2C_MODE_IDLE;
}

// -----------------------------------------------------------------------------
// Initiate or continue transmission from a buffer.
static void I2CTxBuffer(void)
{
  TWDR = *tx_source_ptr_;
  tx_source_ptr_++;
  tx_source_len_--;
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
}

// -----------------------------------------------------------------------------
// Initiate transmission of a single byte.
static void I2CTxByte(uint8_t byte)
{
  TWDR = byte;
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
}

// -----------------------------------------------------------------------------
// Go to the next communication phase.
static void Next(void)
{
  if (i2c_mode_ == I2C_MODE_TX_THEN_RX && i2c_error_ == I2C_ERROR_NONE)
  {
    I2CStart(I2C_MODE_RX);
  }
  else
  {
    I2CStop();
    if (callback_ptr_) (*callback_ptr_)();
  }
}

// -----------------------------------------------------------------------------
// Interrupt received during a transmit phase. Process accordingly.
static void ProcessTxInterrupt(void)
{
  switch (TWSR)
  {
    case TW_START:  // Start condition transmitted
    case TW_REP_START:  // Repeated start condition transmitted
      // Send a write request to the desired slave address.
      I2CTxByte(slave_address_ + TW_WRITE);
      break;
    case TW_MT_SLA_ACK:  // SLA+W transmitted, ACK received
    case TW_MT_DATA_ACK:  // Data transmitted, ACK received
      if (tx_source_len_ > 0)
      {
        I2CTxBuffer();
      }
      else
      {
        // Target is erroneously expecting more data
        // i2c_error_ = I2C_ERROR_ACK;
        Next();
      }
      break;
    case TW_MT_DATA_NACK:  // Data transmitted, NACK received
      if (tx_source_len_ > 0)
      {
        // Indicates that target has canceled reception
        i2c_error_ = I2C_ERROR_NACK;
      }
      Next();
      break;
    case TW_MT_SLA_NACK:  // SLA+W transmitted, NACK received
      // Suggests that the target device is not present
      i2c_error_ = I2C_ERROR_NO_REPLY;
      Next();
      break;
    case TW_MT_ARB_LOST:  // arbitration lost in SLA+W or data
    case TW_NO_INFO:  // no state information available
    case TW_BUS_ERROR:  // illegal start or stop condition
    default:
      // Unexpected status message. Send stop.
      i2c_error_ = I2C_ERROR_OTHER;
      Next();
      break;
  }
}

// -----------------------------------------------------------------------------
// Interrupt received during a receive phase. Process accordingly.
static void ProcessRxInterrupt(void)
{
  switch (TWSR)
  {
    case TW_START:  // Start condition transmitted
    case TW_REP_START:  // Repeated start condition transmitted
      // Send a read request to the desired slave address.
      I2CTxByte(slave_address_ + TW_READ);
      break;
    case TW_MR_DATA_ACK:  // Data received, ACK returned
      I2CReadByte();
      // continue
    case TW_MR_SLA_ACK:  // SLA+R transmitted, ACK received
      if (rx_destination_len_ > 1) I2CRxAck();
      else I2CRxNAck();  // Don't send acknowledgment following last reception.
      break;
    case TW_MR_DATA_NACK:  // Data received, NACK returned
      I2CReadByte();
      Next();
      break;
    case TW_MR_SLA_NACK:  // SLA+R transmitted, NACK received
      // Suggests that the target device is not present
      i2c_error_ = I2C_ERROR_NO_REPLY;
      Next();
      break;
    case TW_MR_ARB_LOST:  // Arbitration lost in SLA+R or NACK
    case TW_NO_INFO:  // No state information available
    case TW_BUS_ERROR:  // Illegal start or stop condition
    default:
      // Unexpected status message. Send stop.
      i2c_error_ = I2C_ERROR_OTHER;
      Next();
      break;
  }
}

// -----------------------------------------------------------------------------
// I2C interrupt indicating that the I2C is active and waiting for the next
// instruction.
ISR(TWI_vect)
{
  switch (i2c_mode_)
  {
    case I2C_MODE_TX:
    case I2C_MODE_TX_THEN_RX:
      ProcessTxInterrupt();
      break;
    case I2C_MODE_RX:
      ProcessRxInterrupt();
      break;
    default:
      Next();  // Unexpected interrupt, reset interrupt flag;
      break;
  }
}
