#ifndef ATTITUDE_H_
#define ATTITUDE_H_


#include <inttypes.h>

#include "main.h"


// =============================================================================
// Accessors:

float * GravityInBodyVector(void);

// -----------------------------------------------------------------------------
float HeadingAngle(void);

// -----------------------------------------------------------------------------
float * Quat(void);


// =============================================================================
// Public functions:

void ResetAttitude(void);

// -----------------------------------------------------------------------------
void UpdateAttitude(void);


#endif  // ATTITUDE_H_
