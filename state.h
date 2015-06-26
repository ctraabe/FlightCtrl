#ifndef STATE_H_
#define STATE_H_


#include <inttypes.h>


enum StateBits {
  STATE_BIT_MOTORS_INHIBITED   = 1<<0,
  STATE_BIT_INITIALIZED        = 1<<1,
  STATE_BIT_STARTING           = 1<<2,
  STATE_BIT_MOTORS_RUNNING     = 1<<3,
  STATE_BIT_IN_AIR             = 1<<4,
  STATE_BIT_LOW_BATTERY        = 1<<5,
  STATE_BIT_EMERGENCY_LANDING  = 1<<6,
};

// =============================================================================
// Accessors:

enum StateBits State(void);


// =============================================================================
// Public functions:

uint8_t MotorsRunning(void);

// -----------------------------------------------------------------------------
void UpdateState(void);


#endif  // STATE_H_
