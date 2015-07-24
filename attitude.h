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

// TODO: REMOVE*****************************************************************
void SetResetAttitude(void);
uint8_t GetResetAttitude(void);
uint8_t GetDebugResetAttitude(void);


// =============================================================================
// Public functions:

void ResetAttitude(void);

// -----------------------------------------------------------------------------
void UpdateAttitude(void);


#endif  // ATTITUDE_H_
