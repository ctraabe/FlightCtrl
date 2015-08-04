#include "mk_serial_rx.h"

#include "mk_serial_protocol.h"
#include "mk_serial_tx.h"
#include "timing.h"


// =============================================================================
// Public functions:

// This function handles the response to data that has been received in the
// MikroKopter protocol.
void HandleMKRx(uint8_t address, uint8_t label, uint8_t * data_buffer)
{
  // First check for the following address independent messages.
  switch (label)
  {
    case 'i':  // Request MK data stream.
      SetMKDataStream(MK_STREAM_CONTROL, 0);
      break;
    case 'd':  // Request MK debug stream.
      SetMKDataStream(MK_STREAM_DEBUG, data_buffer[0]);
      break;
    case 'v': // Version requirement and expansion stage
      SetMKTxRequest(MK_TX_VERSION);
      break;
    default:
      // Check for FlightCtrl specific messages.
      if (address == MK_SERIAL_FC_ADDRESS)
      {
      }
      break;
  }
}
