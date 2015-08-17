// Information:
//
// 57600 BAUD with one start and end bit gives a maximum data rate of 5.76
// kBytes / second. However, MikroKopter uses an additional protocol to manage
// the data transfer. In this protocol, multiple bytes of data can be wrapped
// into a single frame with some identifying information. The identifying
// information includes the following at the start of the frame:
//
//   - the start character "#"
//   - an address
//   - a label identifier
//
// and the following at the end of the frame:
//
//   - 2 bytes containing the value for the checksum
//   - the end character "\r"
//
// Finally, to guarantee identification of the start and end characters, data is
// separated into 3-byte chunks and each chunk is re-encoded into 4-bytes. Each
// of the encoded bytes has a minimum value of 61, so that it cannot be confused
// with the start character "#" (35) or the end character "\r" (13).
//
// Therefore, here is a summary of the total overhead:
//
//   - 1 extra byte per 3-bytes of data in a frame for encoding
//   - 4 extra bytes per data frame for identification and error checking
//   - 2 extra bits per byte to mark the start and end of a transmitted byte
//
// Here are some example maximum transfer rates for certain data sizes:
//
//   1-3 bytes -> 576 Hz
//   4-6 bytes -> 411 Hz
//   7-9 bytes -> 320 Hz
//   22-24 bytes -> 151 Hz
//   25-27 bytes -> 137 Hz
//   28-30 bytes -> 125 Hz
//   66 bytes -> 61.3 Hz (this is the size of MikroKopter's DebugOut structure)

#include "mk_serial_protocol.h"

#include "mk_serial_rx.h"
#include "union_types.h"


// =============================================================================
// Private function declarations:

static void DecodeMKSerialRx(uint8_t * data_buffer, uint8_t length);;
static void AddMKChecksum(uint8_t * buffer, uint8_t length);
static uint8_t VerifyMKChecksum(uint8_t * data_buffer, uint8_t length);


// =============================================================================
// Public functions:

// This function collects an incoming byte that is assumed to be part of a
// message encoded in the MikroKopter protocol. This function attempts to place
// the incoming byte into the shared data buffer, but abandons the reception if
// the data buffer is not large enough. The return value indicates whether or
// not more bytes are expected. If so, subsequent bytes should also be passed to
// this function.
enum UARTRxMode MKSerialRx(uint8_t byte, uint8_t * data_buffer)
{
  static uint8_t length = 0;

  if (byte != '\r')  // The byte '/r' marks the end of an MK message
  {
    if (length < DATA_BUFFER_LENGTH)
    {
      data_buffer[length++] = byte;
      return UART_RX_MODE_MK_ONGOING;
    }
  }
  else if (VerifyMKChecksum(data_buffer, length))
  {
    DecodeMKSerialRx(data_buffer, length);
  }
  length = 0;
  return UART_RX_MODE_IDLE;
}

// -----------------------------------------------------------------------------
// This function encodes data into a message using the MikroKopter protocol.
// The message must contain at least a destination address and a label. If no
// additional data is necessary, then the source pointer and length can both be
// set to zero.
void MKSerialTx(uint8_t address, uint8_t label, const uint8_t * source,
  uint8_t length)
{
  uint8_t * tx_buffer = RequestUARTTxBuffer();
  if (!tx_buffer) return;

  uint8_t i = 0;
  tx_buffer[i++] = '#';
  tx_buffer[i++] = 'a' + address;
  tx_buffer[i++] = label;

  while (length)
  {
    uint8_t y[3] = { 0 };
    for (uint8_t j = 3; j-- && length--; ) y[j] = *(source++);

    tx_buffer[i++] = '=' + (y[2] >> 2);
    tx_buffer[i++] = '=' + (((y[2] & 0x03) << 4) | ((y[1] & 0xf0) >> 4));
    tx_buffer[i++] = '=' + (((y[1] & 0x0f) << 2) | ((y[0] & 0xc0) >> 6));
    tx_buffer[i++] = '=' + (y[0] & 0x3f);
  }

  AddMKChecksum(tx_buffer, i);
  i += sizeof(uint16_t);

  tx_buffer[i++] = '\r';

  UARTTxBuffer(i);
}


// =============================================================================
// Private functions:

// This function extracts the data from a message encoded in the MikroKopter
// protocol. Here, length excludes the start and end characters only.
static void DecodeMKSerialRx(uint8_t * data_buffer, uint8_t length)
{
  uint8_t address = data_buffer[0] - 'a';
  uint8_t label = data_buffer[1];

  uint8_t in = 2, out = 0;
  while (in < length - 2)
  {
    uint8_t a, b, c, d;

    a = data_buffer[in++] - '=';
    b = data_buffer[in++] - '=';
    c = data_buffer[in++] - '=';
    d = data_buffer[in++] - '=';

    a = (a << 2) | (b >> 4);
    b = ((b & 0x0F) << 4) | (c >> 2);
    c = ((c & 0x03) << 6) | d;

    data_buffer[out++] = a;
    data_buffer[out++] = b;
    data_buffer[out++] = c;
  }

  HandleMKRx(address, label, data_buffer);
}

// -----------------------------------------------------------------------------
// This function computes the MikroKopter checksum for encoded data. Here,
// length excludes the start and end characters and the checksum.
static uint16_t MKChecksum(const uint8_t * buffer, uint8_t length)
{
  union U16Bytes result = { '#' };
  for (uint8_t i = length; i--; ) result.u16 += buffer[i];

  result.bytes[1] &= 0x0F;  // checksum % 4096
  uint8_t temp = result.bytes[0] & 0x3F;  // checksum % 64
  result.u16 >>= 6;  // checksum / 64
  result.bytes[0] += '=';
  result.bytes[1] = temp + '=';

  return result.u16;
}

// -----------------------------------------------------------------------------
// This function appends the MikroKopter checksum to the end of some encoded
// data. Here, length excludes the start and end characters and the checksum.
static void AddMKChecksum(uint8_t * buffer, uint8_t length)
{
  uint16_t * checksum_ptr = (uint16_t *)&buffer[length];
  *checksum_ptr = MKChecksum(++buffer, --length);
}

// -----------------------------------------------------------------------------
// This function verifies the checksum of some received data. Here, length
// excludes the start and end characters only.
static uint8_t VerifyMKChecksum(uint8_t * data_buffer, uint8_t length)
{
  uint16_t * checksum_ptr = (uint16_t *)&data_buffer[length-2];
  return *checksum_ptr == MKChecksum(data_buffer, length - 2);
}
