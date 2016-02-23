#ifndef STATE_H_
#define STATE_H_


#include <inttypes.h>


enum StateBits {
  STATE_BIT_MOTORS_INHIBITED      = 1<<0,
  STATE_BIT_INITIALIZED           = 1<<1,
  STATE_BIT_STARTING              = 1<<2,
  STATE_BIT_MOTORS_RUNNING        = 1<<3,
};


// =============================================================================
// Accessors:

uint8_t AltitudeControlActive(void);

// -----------------------------------------------------------------------------
uint8_t MotorsInhibited(void);

// -----------------------------------------------------------------------------
uint8_t MotorsRunning(void);

// -----------------------------------------------------------------------------
enum NavMode NavModeRequest(void);

// -----------------------------------------------------------------------------
enum StateBits State(void);

// -----------------------------------------------------------------------------
uint8_t Takeoff(void);


// =============================================================================
// Public functions:

void UpdateState(void);


#endif  // STATE_H_
