#ifndef MOTORS_H_
#define MOTORS_H_


#include <inttypes.h>


#define MOTORS_MAX (8)

enum BLCErrorBits {
  BLC_ERROR_BIT_MISSING_MOTOR = 1<<0,
  BLC_ERROR_BIT_EXTRA_MOTOR = 1<<1,
  BLC_ERROR_BIT_INCONSISTENT_SETTINGS = 1<<2,
};


// =============================================================================
// Accessors

uint8_t BLCErrorBits(void);

// -----------------------------------------------------------------------------
uint8_t NMotors(void);


// =============================================================================
// Public functions:

void DetectMotors(void);

// -----------------------------------------------------------------------------
uint8_t MotorsStarting(void);

// -----------------------------------------------------------------------------
void SetMotorSetpoint(uint8_t address, uint16_t setpoint);

// -----------------------------------------------------------------------------
void SetNMotors(uint8_t n_motors);

// -----------------------------------------------------------------------------
void TxMotorSetpoints(void);


#endif  // MOTORS_H_
