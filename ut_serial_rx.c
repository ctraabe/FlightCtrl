#include "ut_serial_rx.h"

#include "buzzer.h"
#include "ut_serial_protocol.h"


// =============================================================================
// Public functions:

// This function handles the response to data that has been received in the
// UTokyo protocol.
void HandleUTRx(uint8_t id, const uint8_t * data_buffer)
{
  switch (id)
  {
    case UT_SERIAL_ID_BEEP_PATTERN:
      BeepPattern(((uint16_t *)data_buffer)[0]);
      break;
    default:
      break;
  }
}
