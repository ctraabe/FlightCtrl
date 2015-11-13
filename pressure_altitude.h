#ifndef PRESSURE_ALTITUDE_H_
#define PRESSURE_ALTITUDE_H_


#include <inttypes.h>


enum PressureAltitudeErrorBits {
  PRESSURE_ERROR_BIT_BIAS_RANGE = 1<<0,
  PRESSURE_ERROR_BIT_COARSE_CALIBRATION = 1<<1,
  PRESSURE_ERROR_BIT_FINE_CALIBRATION = 1<<2,
};


// =============================================================================
// Public functions:

// TIMER0 is used to drive the PWM signals that set the coarse and fine biases
// for the pressure sensor.
void PressureSensorInit(void);

// -----------------------------------------------------------------------------
// This function adjusts the duty cycle of the PWM signals that are used to bias
// the output of the pressure sensor amplifier. This effectively shifts the 140
// m measurable range to encompass the starting altitude (see explanation at the
// top of this file).
void ResetPressureSensorRange(void);

// -----------------------------------------------------------------------------
// This function attempts to calibrate the bias for more accurate shifts in
// the pressure altitude measurement range. Deviation from the theoretical
// relationship is possible due to imprecise resistor values.
void PressureSensorBiasCalibration(void);

// -----------------------------------------------------------------------------
void UpdatePressureAltitude(void);


#endif  // PRESSURE_ALTITUDE_H_
