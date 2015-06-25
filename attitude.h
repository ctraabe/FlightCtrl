#ifndef ATTITUDE_H_
#define ATTITUDE_H_


#include <inttypes.h>

#include "global_definitions.h"


// =============================================================================
// Accessors:

float Gravity(enum BodyAxes axis);

// -----------------------------------------------------------------------------
float Quat(uint8_t n);


// =============================================================================
// Public functions:

void UpdateAttitude(void);

// -----------------------------------------------------------------------------
float HeadingAngle(void);


#endif  // ATTITUDE_H_
