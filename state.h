#ifndef STATE_H_
#define STATE_H_


#include <inttypes.h>


enum StateBits {
  STATE_BIT_MOTORS_INHIBITED           = 1<<0,
  STATE_BIT_INITIALIZED                = 1<<1,
  STATE_BIT_STARTING                   = 1<<2,
  STATE_BIT_MOTORS_RUNNING             = 1<<3,
  STATE_BIT_HORIZONTAL_POSITION_INHIBITED = 1<<4,
  STATE_BIT_VERTICAL_POSITION_INHIBITED = 1<<5,
  STATE_BIT_TAKEOFF                    = 1<<6,
};

enum HorizontalControlState {
  HORIZONTAL_CONTROL_STATE_MANUAL = 0,
  HORIZONTAL_CONTROL_STATE_HOLD,
  HORIZONTAL_CONTROL_STATE_AUTO,
  HORIZONTAL_CONTROL_STATE_TAKEOFF,
};

enum VerticalControlState {
  VERTICAL_CONTROL_STATE_MANUAL = 0,
  VERTICAL_CONTROL_STATE_BARO,
  VERTICAL_CONTROL_STATE_AUTO,
};


// =============================================================================
// Accessors:

enum StateBits State(void);

// -----------------------------------------------------------------------------
enum HorizontalControlState HorizontalControlState(void);

// -----------------------------------------------------------------------------
enum VerticalControlState VerticalControlState(void);

// -----------------------------------------------------------------------------
uint8_t MotorsInhibited(void);

// -----------------------------------------------------------------------------
uint8_t MotorsRunning(void);


// =============================================================================
// Public functions:

void UpdateState(void);


#endif  // STATE_H_
