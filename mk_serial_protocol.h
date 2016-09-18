#ifndef MK_SERIAL_PROTOCOL_H_
#define MK_SERIAL_PROTOCOL_H_


#include <inttypes.h>

#include "uart.h"


#define MK_START_CHARACTER ('#')

#define MK_SERIAL_ALL_ADDRESS (0)
#define MK_SERIAL_FC_ADDRESS (1)
#define MK_SERIAL_NC_ADDRESS (2)
#define MK_SERIAL_MK3MAG_ADDRESS (3)
#define MK_SERIAL_BL_CTRL_ADDRESS (5)


// =============================================================================
// Public functions:

// This function collects an incoming byte that is assumed to be part of a
// message encoded in the MikroKopter protocol. The return value indicates
// whether or not more bytes are expected. If so, subsequent bytes should also
// be passed to this function.
enum UARTRxMode MKSerialRx(uint8_t byte, uint8_t * data_buffer);

// -----------------------------------------------------------------------------
// This function encodes data into a message using the MikroKopter protocol.
// The message must contain at least a destination address and a label. If no
// additional data is necessary, then the source pointer and length can both be
// set to zero.
void MKSerialTx(uint8_t address, uint8_t command, const uint8_t * source,
  uint8_t length);


#endif  // MK_SERIAL_PROTOCOL_H_
