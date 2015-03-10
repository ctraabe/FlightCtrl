#ifndef ADC_H_
#define ADC_H_


// =============================================================================
// Definitions:

// ADC_N_SAMPLES defines the size of the ADC sample array. It is the number of
// samples that are combined to report a sensor reading. Increasing this number
// will improve fidelity and noise rejection, but will also increase latency.
// Due to the structure of the ADC interrupt handler this number must be a power
// of 2 between 1 and 32 (inclusively). The ADC sample array will be fully
// refreshed at 20,000,000 / 128 / 13 / (ADC_N_CHANNELS * ADC_N_SAMPLES) Hz.
#define ADC_N_SAMPLES (1 << 3)  // 8
#define ADC_N_CHANNELS (8)  // Do not modify!!!


#ifndef __ASSEMBLER__


#include <inttypes.h>


enum SensorAxes {
  X_AXIS = 0,
  Y_AXIS = 1,
  Z_AXIS = 2,
};

enum ADCState {
  ADC_INACTIVE = 0,
  ADC_ACTIVE = 1,
};


// =============================================================================
// Accessors:

int16_t Acceleration(enum SensorAxes axis);

// -----------------------------------------------------------------------------
uint16_t Accelerometer(enum SensorAxes axis);

// -----------------------------------------------------------------------------
enum ADCState ADCState(void);

// -----------------------------------------------------------------------------
int16_t AngularRate(enum SensorAxes axis);

// -----------------------------------------------------------------------------
uint16_t BaroAltitude(void);

// -----------------------------------------------------------------------------
uint16_t BatteryVoltage(void);

// -----------------------------------------------------------------------------
uint16_t Gyro(enum SensorAxes axis);

// -----------------------------------------------------------------------------
uint16_t PressureSensor(void);


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
