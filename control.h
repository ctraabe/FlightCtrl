#ifndef CONTROL_H_
#define CONTROL_H_


#include <inttypes.h>

#include "motors.h"


// =============================================================================
// Public functions:

void ControlInit(void);

// -----------------------------------------------------------------------------
void Control(void);

// -----------------------------------------------------------------------------
void SetActuationInverse(float actuation_inverse[MOTORS_MAX][4]);


#endif  // CONTROL_H_
