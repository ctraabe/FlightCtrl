#ifndef ATTITUDE_H_
#define ATTITUDE_H_


#include <inttypes.h>

#include "global_definitions.h"


// =============================================================================
// Accessors:

float * GravityInBodyVector(void);

// -----------------------------------------------------------------------------
float * Quat(void);


// =============================================================================
// Public functions:

void ResetAttitude(void);

// -----------------------------------------------------------------------------
void UpdateAttitude(void);

// -----------------------------------------------------------------------------
float HeadingAngle(void);


#endif  // ATTITUDE_H_
