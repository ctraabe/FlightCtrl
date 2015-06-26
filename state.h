#ifndef STATE_H_
#define STATE_H_


enum StateBits {
  STATE_BIT_MOTORS_INHIBITED   = 1<<0,
  STATE_BIT_INITIALIZED        = 1<<1,
  STATE_BIT_STARTED            = 1<<2,
  STATE_BIT_MOTORS_ON          = 1<<3,
  STATE_BIT_IN_AIR             = 1<<4,
  STATE_BIT_LOW_BATTERY        = 1<<5,
  STATE_BIT_EMERGENCY_LANDING  = 1<<6,
};

// =============================================================================
// Accessors:


// =============================================================================
// Public functions:

void UpdateState(void);


#endif  // STATE_H_
