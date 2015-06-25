#ifndef ADC_H_
#define ADC_H_


// ADC_N_SAMPLES defines the size of the ADC sample array. It is the number of
// samples that are combined to report a sensor reading. Increasing this number
// will improve fidelity and noise rejection, but will also increase latency.
// Due to the structure of the ADC interrupt handler this number must be a power
// of 2 between 1 and 32 (inclusively). The ADC sample array will be fully
// refreshed at 20,000,000 / 128 / 13 / (ADC_N_CHANNELS * ADC_N_SAMPLES) Hz.
#define ADC_N_SAMPLES_POW_OF_2 (3)  // 2^3 = 8
#define ADC_N_SAMPLES (1 << ADC_N_SAMPLES_POW_OF_2)  // 8
#define ADC_N_CHANNELS (8)  // Do not modify!!!

#ifndef __ASSEMBLER__


#include <inttypes.h>

#include "global_definitions.h"


enum ADCState {
  ADC_INACTIVE = 0,
  ADC_ACTIVE = 1,
};


// =============================================================================
// Accessors:

float Acceleration(enum BodyAxes axis);

// -----------------------------------------------------------------------------
float * AccelerationVector(void);

// -----------------------------------------------------------------------------
// Returns the most recent accelerometer reading. Scale is 5/1024 g/LSB.
uint16_t Accelerometer(enum BodyAxes axis);

// -----------------------------------------------------------------------------
enum ADCState ADCState(void);

// -----------------------------------------------------------------------------
float AngularRate(enum BodyAxes axis);

// -----------------------------------------------------------------------------
uint16_t BatteryVoltage(void);

// -----------------------------------------------------------------------------
uint16_t BiasedPressure(void);

// -----------------------------------------------------------------------------
uint16_t BiasedPressureSensor(void);

// -----------------------------------------------------------------------------
// Returns the most recent gyro reading. Scale is 5/6.144 deg/s/LSB.
uint16_t Gyro(enum BodyAxes axis);

// -----------------------------------------------------------------------------
// This function assumes that the Mikrokopter is motionless on the ground. It
// finds the average gyro readings over the period of 1 second (approximately)
// and considers the results to be the zero values of the gyros.
void ZeroGyros(void);

// -----------------------------------------------------------------------------
// This function assumes that the Mikrokopter is motionless (on the ground). It
// finds the average accelerometer readings over the period of 1 second
// (approximately) and considers the results to be the zero values of the
// accelerometers.
void ZeroAccelerometers(void);

// -----------------------------------------------------------------------------
// This function loads the accelerometer offsets from EEPROM so that
// accelerometers don't have to be re-calibrated every flight.
void LoadAccelerometerOffsets(void);


// =============================================================================
// Public functions:

// This function starts the ADC in free-running mode.
void ADCOn(void);

// -----------------------------------------------------------------------------
// This function immediately kills the ADC.
void ADCOff(void);

// -----------------------------------------------------------------------------
// This function sums several sensor readings (each reading the sample array) in
// order to increase fidelity.
void ProcessSensorReadings(void);

// -----------------------------------------------------------------------------
// This function delays program execution until the ADC sample array has been
// fully refreshed at least once.
void WaitOneADCCycle(void);


#endif  // __ASSEMBLER__

#endif  // ADC_H_
