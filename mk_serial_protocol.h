#ifndef MK_SERIAL_PROTOCOL_H_
#define MK_SERIAL_PROTOCOL_H_


#include <inttypes.h>

#include "uart.h"


#define MK_SERIAL_FC_ADDRESS (1)
#define MK_SERIAL_NC_ADDRESS (2)
#define MK_SERIAL_MK3MAG_ADDRESS (3)
#define MK_SERIAL_BL_CTRL_ADDRESS (5)


// =============================================================================
// Public functions:

enum UARTRxMode MKSerialRx(uint8_t byte, uint8_t * data_buffer);

// -----------------------------------------------------------------------------
void MKSerialTx(uint8_t address, uint8_t command, uint8_t * source,
  uint8_t length);


#endif  // MK_SERIAL_PROTOCOL_H_
