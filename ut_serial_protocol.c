#include "ut_serial_protocol.h"

#include <string.h>
#include <util/crc16.h>

#include "union_types.h"
#include "ut_serial_rx.h"


// =============================================================================
// Public functions:

// This function collects an incoming byte that is assumed to be part of a
// message encoded in the UTokyo protocol. This function attempts to place the
// incoming byte into the shared data buffer, but abandons the reception if the
// data buffer is not large enough. The return value indicates whether or not
// more bytes are expected. If so, subsequent bytes should also be passed to
// this function.
enum UARTRxMode UTSerialRx(uint8_t byte, uint8_t * data_buffer)
{
  static uint8_t * rx_ptr = 0;
  static uint8_t bytes_processed = 0, length = 0, id = 0;
  static union U16Bytes crc;

  if (bytes_processed == 0)  // First byte is payload length
  {
    if ((UT_HEADER_LENGTH + byte) > UART_DATA_BUFFER_LENGTH) goto RESET;
    length = byte;
    crc.u16 = _crc_ccitt_update(0xFFFF, byte);
    rx_ptr = data_buffer;
  }
  else if (bytes_processed == 1)  // Second byte is id
  {
    id = byte;
    crc.u16 = _crc_ccitt_update(crc.u16, byte);
  }
  else if (bytes_processed == 2)  // Third byte is currently unused
  {
    crc.u16 = _crc_ccitt_update(crc.u16, byte);
  }
  else if (bytes_processed < (UT_HEADER_LENGTH - 1 + length))  // Payload
  {
    crc.u16 = _crc_ccitt_update(crc.u16, byte);
    *rx_ptr++ = byte;
  }
  else if (bytes_processed == (UT_HEADER_LENGTH - 1 + length))  // CRC[0]
  {
    if (byte != crc.bytes[0]) goto RESET;
  }
  else  // CRC[1]
  {
    if (byte == crc.bytes[1]) HandleUTRx(id, data_buffer);
    goto RESET;
  }
  bytes_processed++;
  return UART_RX_MODE_UT_ONGOING;

  RESET:
  bytes_processed = 0;
  return UART_RX_MODE_IDLE;
}

// -----------------------------------------------------------------------------
// This function encodes data into a message using the UTokyo protocol. The
// message must contain at least a destination address and a label. If no
// additional data is necessary, then the source pointer and length can both be
// set to zero.
void UTSerialTx(uint8_t id, const uint8_t * source, uint8_t length)
{
  if ((length + 1 + UT_HEADER_LENGTH + 2) > UART_TX_BUFFER_LENGTH) return;

  uint8_t * tx_buffer = RequestUARTTxBuffer();
  if (!tx_buffer) return;
  uint8_t * tx_ptr = tx_buffer;

  // Copy the start character to the TX buffer;
  *tx_ptr++ = UT_START_CHARACTER;

  // Copy the payload length to the TX buffer.
  *tx_ptr++ = length;

  // Copy the id to the TX buffer.
  *tx_ptr++ = id;

  // The next byte is unused (for padding and future use)
  tx_ptr++;

  // Copy the payload to the TX buffer.
  memcpy(tx_ptr, source, length);
  tx_ptr += length;

  // Compute the CRC (starting from payload length) and copy to the TX buffer.
  union U16Bytes crc = { 0xFFFF };
  for(uint8_t i = 1; i < length + UT_HEADER_LENGTH; ++i)
    crc.u16 = _crc_ccitt_update(crc.u16, tx_buffer[i]);
  *tx_ptr++ = crc.bytes[0];
  *tx_ptr = crc.bytes[1];

  UARTTxBuffer(length + UT_HEADER_LENGTH + 2);
}
