#ifndef NAV_COMMS_H_
#define NAV_COMMS_H_


#include <inttypes.h>


// =============================================================================
// Accessors:

uint8_t NavDataReady(void);

// -----------------------------------------------------------------------------
uint8_t NavRecieved(void);

// -----------------------------------------------------------------------------
const volatile float * Position(void);

// -----------------------------------------------------------------------------
const volatile float * Velocity(void);

// -----------------------------------------------------------------------------
float HeadingCorrection(void);


// =============================================================================
// Public functions:

void NavCommsInit(void);

// -----------------------------------------------------------------------------
void ExchangeDataWithNav(float accel);

// -----------------------------------------------------------------------------
void NotifyNav(void);

// -----------------------------------------------------------------------------
void ProcessDataFromNav(void);


#endif  // NAV_COMMS_H_
