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

// -----------------------------------------------------------------------------
float * UpdateGravtiyInBody(float quat[4], float g_b[3]);

// -----------------------------------------------------------------------------
float * UpdateQuaternion(float quat[4], float angular_rate[3], float dt);


#endif  // ATTITUDE_H_
