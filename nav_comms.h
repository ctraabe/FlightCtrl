#ifndef NAV_COMMS_H_
#define NAV_COMMS_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

void NavCommsInit(void);

// -----------------------------------------------------------------------------
void NotifyNav(void);

// -----------------------------------------------------------------------------
void SendDataToNav(void);


#endif  // NAV_COMMS_H_
