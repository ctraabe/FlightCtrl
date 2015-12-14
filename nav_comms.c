#include "nav_comms.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/crc16.h>

#include "adc.h"
#include "attitude.h"
#include "main.h"
#include "spi.h"
#include "timing.h"
#include "union_types.h"
// TODO: remove
#include "led.h"


// =============================================================================
// Private data:

#define NAV_MESSAGE_START_BYTE (0xAA)

enum NavCommsState {
  NAV_COMMS_IDLE = 0,
  NAV_COMMS_DATA_READY,
  NAV_COMMS_DATA_RECEVIED,
};

static volatile struct FromNav {
  float position[3];
  float velocity[3];
  float heading_correction;
  uint16_t crc;
} __attribute__((packed)) from_nav_[2];

static volatile uint8_t nav_data_state_ = NAV_COMMS_IDLE;
static uint8_t from_nav_head_ = 1, from_nav_tail_ = 0;


// =============================================================================
// Private function declarations:

static void SetNavDataReceived(void);


// =============================================================================
// Accessors:

uint8_t NavDataReady(void)
{
  return nav_data_state_ == NAV_COMMS_DATA_READY;
}

// -----------------------------------------------------------------------------
uint8_t NavRecieved(void)
{
  return nav_data_state_ == NAV_COMMS_DATA_RECEVIED;
}

// -----------------------------------------------------------------------------
const volatile float * Position(void)
{
  return from_nav_[from_nav_tail_].position;
}

// -----------------------------------------------------------------------------
const volatile float * Velocity(void)
{
  return from_nav_[from_nav_tail_].velocity;
}

// -----------------------------------------------------------------------------
float HeadingCorrection(void)
{
  return from_nav_[from_nav_tail_].heading_correction;
}


// =============================================================================
// Public functions:

void NavCommsInit(void)
{
  // Pull up pin PC4.
  PORTC |= _BV(4);

  // Enable pin change interrupts.
  PCICR |= _BV(PCIE2);
}

// -----------------------------------------------------------------------------
void NotifyNav(void)
{
  // Disable the pin change interrupt for PC4.
  PCMSK2 &= ~_BV(PCINT20);

  // Pull down PC4.
  PORTC &= ~_BV(4);
  DDRC |= _BV(4);
}

// -----------------------------------------------------------------------------
void ProcessDataFromNav(void)
{
  uint8_t * rx_buffer_ptr = (uint8_t *)&from_nav_[from_nav_head_];
  uint16_t crc = 0xFFFF;
  for (uint8_t i = sizeof(struct FromNav) - 2; i--; )
    crc = _crc_ccitt_update(crc, *(rx_buffer_ptr++));

  if (from_nav_[from_nav_head_].crc == crc)
  {
    // Swap buffers.
    from_nav_tail_ = from_nav_head_;
    from_nav_head_ = !from_nav_tail_;
  }

  // RedLEDOff();
  nav_data_state_ = NAV_COMMS_IDLE;
}

// -----------------------------------------------------------------------------
void ReceiveDataFromNav(void)
{
  SPIRxThenCallback((uint8_t *)&from_nav_[from_nav_head_],
    sizeof(struct FromNav), SetNavDataReceived);
}

// -----------------------------------------------------------------------------
void SendDataToNav(void)
{
  // Set PC4 back to input and pull-up.
  DDRC &= ~_BV(4);
  PORTC |= _BV(4);

  // Request the SPI transmit buffer. Return if not available.
  uint8_t * tx_buffer = RequestSPITxBuffer();
  if (tx_buffer == 0) return;

  // Specify the payload structure.
  struct ToNav {
    float acceleration[3];
    float angular_rate[3];
    float quaternion[4];
  } __attribute__((packed));
  _Static_assert(sizeof(struct ToNav) + 4 < SPI_TX_BUFFER_LENGTH,
    "struct ToNav is too large for the SPI TX buffer");

  // Populate the transmit buffer.
  tx_buffer[0] = NAV_MESSAGE_START_BYTE;
  tx_buffer[1] = sizeof(struct ToNav);

  struct ToNav * to_nav_ptr = (struct ToNav *)&tx_buffer[2];
  to_nav_ptr->acceleration[0] = Acceleration(X_BODY_AXIS);
  to_nav_ptr->acceleration[1] = Acceleration(Y_BODY_AXIS);
  to_nav_ptr->acceleration[2] = Acceleration(Z_BODY_AXIS);
  to_nav_ptr->angular_rate[0] = AngularRate(X_BODY_AXIS);
  to_nav_ptr->angular_rate[1] = AngularRate(Y_BODY_AXIS);
  to_nav_ptr->angular_rate[2] = AngularRate(Z_BODY_AXIS);
  to_nav_ptr->quaternion[0] = Quat()[0];
  to_nav_ptr->quaternion[1] = Quat()[1];
  to_nav_ptr->quaternion[2] = Quat()[2];
  to_nav_ptr->quaternion[3] = Quat()[3];

  // Add a CRC just after the data payload. Note that this is the same CRC that
  // is used in MAVLink. It uses a polynomial represented by 0x1021, an initial
  // value of 0xFFFF, and with both the input and output reflected.
  union U16Bytes * crc = (union U16Bytes *)&tx_buffer[2 + sizeof(struct ToNav)];
  crc->u16 = 0xFFFF;
  uint8_t * tx_buffer_ptr = &tx_buffer[1];  // Skip the start byte
  for (uint8_t i = sizeof(struct ToNav) + 1; i--; )
    crc->u16 = _crc_ccitt_update(crc->u16, *tx_buffer_ptr++);

  // The MikroKopter NaviCtrl board employs a STR91x microcontroller. The SPI Rx
  // interrupt for this board only occurs when the Rx buffer (8 bytes) is more
  // than half full. To make sure that this interrupt is triggered, add 4
  // trailing bytes to the end of the transmission.
  tx_buffer_ptr += sizeof(uint16_t);  // Skip the CRC
  for (uint8_t i = 4; i--; ) *tx_buffer_ptr++ = 0x00;

  // Send the entire packet (1-byte header, 1-byte length, payload, 2-byte CRC &
  // 4-bytes trailing zeros).
  SPITxBuffer(2 + sizeof(struct ToNav) + 2 + 4);

  // Re-enable the pin change interrupt for pin PC4.
  PCMSK2 |= _BV(PCINT20);
}


// =============================================================================
// Private function:

static void SetNavDataReceived(void)
{
  nav_data_state_ = NAV_COMMS_DATA_RECEVIED;
}

// -----------------------------------------------------------------------------
ISR(PCINT2_vect)
{
  // Ignore rising edge.
  if (PINC & _BV(4)) return;

  nav_data_state_ = NAV_COMMS_DATA_READY;
  // RedLEDOn();

  // Disable the interrupt.
  PCMSK2 &= ~_BV(PCINT20);
}