#include "nav_comms.h"

#include <util/crc16.h>

#include "adc.h"
#include "attitude.h"
#include "main.h"
#include "spi.h"


// =============================================================================
// Private data:

#define NAV_MESSAGE_START_BYTE (0xFE)


// =============================================================================
// Private function declarations:


// =============================================================================
// Public functions:

void SendDataToNav(void)
{
  uint8_t * tx_buffer = RequestSPITxBuffer();

  struct ToNav {
    uint8_t header;
    float acceleration[3];
    float angular_rate[3];
    float quaternion[4];
    uint16_t crc;
  } __attribute__((packed));

  _Static_assert(sizeof(struct ToNav) + 4 < SPI_TX_BUFFER_LENGTH,
    "struct ToNav is too large for the SPI TX buffer");

  struct ToNav * to_nav_ptr = (struct ToNav *)&tx_buffer[0];

  to_nav_ptr->header = NAV_MESSAGE_START_BYTE;
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

  // Add a CRC (same version used in MAVLink).
  to_nav_ptr->crc = 0xFFFF;
  const uint8_t * tx_buffer_ptr = &tx_buffer[0];
  for (uint8_t i = sizeof(struct ToNav) - sizeof(uint16_t); i--; )
  {
    to_nav_ptr->crc = _crc_ccitt_update(to_nav_ptr->crc, *tx_buffer_ptr++);
  }

  // Add 4 trailing zeros to force STR91x SPI Rx interrupt.
  tx_buffer[sizeof(struct ToNav) + 0] = 0x00;
  tx_buffer[sizeof(struct ToNav) + 1] = 0x00;
  tx_buffer[sizeof(struct ToNav) + 2] = 0x00;
  tx_buffer[sizeof(struct ToNav) + 3] = 0x00;

  SPITxBuffer(sizeof(struct ToNav) + 4);
}
