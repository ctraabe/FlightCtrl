#ifndef MK_SERIAL_RX_H_
#define MK_SERIAL_RX_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

// This function handles the response to data that has been received in the
// MikroKopter protocol.
void HandleMKRx(uint8_t address, uint8_t label, uint8_t * data_buffer);


#endif  // MK_SERIAL_RX_H_
