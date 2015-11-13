#ifndef ALTITUDE_H_
#define ALTITUDE_H_


#include <inttypes.h>


// =============================================================================
// Accessors:

float Altitude(void);

// -----------------------------------------------------------------------------
float VerticalSpeed(void);


// =============================================================================
// Public functions:

void ResetAltitude(void);

// -----------------------------------------------------------------------------
void UpdateAltitude(void);


#endif  // ALTITUDE_H_
