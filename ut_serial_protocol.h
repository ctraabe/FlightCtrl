#ifndef UT_SERIAL_PROTOCOL_H_
#define UT_SERIAL_PROTOCOL_H_


#include <inttypes.h>

#include "uart.h"


#define UT_START_CHARACTER ('S')
#define UT_HEADER_LENGTH (4)

enum UTSerialID {
  UT_SERIAL_ID_BEEP_PATTERN = 0,
};


// =============================================================================
// Public functions:

// This function collects an incoming byte that is assumed to be part of a
// message encoded in the UTokyo protocol. The return value indicates whether or
// not more bytes are expected. If so, subsequent bytes should also be passed to
// this function.
enum UARTRxMode UTSerialRx(uint8_t byte, uint8_t * data_buffer);

// -----------------------------------------------------------------------------
// This function encodes data into a message using the UTokyo protocol. The
// message must contain at least a destination address and a label. If no
// additional data is necessary, then the source pointer and length can both be
// set to zero.
void UTSerialTx(uint8_t id, const uint8_t * source, uint8_t length);


#endif  // UT_SERIAL_PROTOCOL_H_
