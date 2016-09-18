#ifndef UT_SERIAL_RX_H_
#define UT_SERIAL_RX_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

// This function handles the response to data that has been received in the
// UTokyo protocol.
void HandleUTRx(uint8_t id, const uint8_t * data_buffer);


#endif  // UT_SERIAL_RX_H_
