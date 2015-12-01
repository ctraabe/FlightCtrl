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


// =============================================================================
// Private function declarations:


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
  RedLEDOn();
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

ISR(PCINT2_vect)
{
  // TODO: This is a high priority interrupt. This routine should be changed to:
  // if (~PINC & BV(4)) <trigger a low priority interrupt>

  // Ignore rising edge.
  if (PINC & _BV(4)) return;

  // TODO: Move the following functionality to the low-priority interrupt.
  // Disable the interrupt.
  PCMSK2 &= ~_BV(PCINT20);
  RedLEDOff();
}