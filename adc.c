#include "adc.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>


// =============================================================================
// Private data:

// ADC sample indices
enum ADCSensorIndex {
  ADC_ACCEL_Y  = 0,
  ADC_ACCEL_X  = 1,
  ADC_GYRO_Z   = 2,
  ADC_GYRO_X   = 3,
  ADC_GYRO_Y   = 4,
  ADC_PRESSURE = 5,
  ADC_BATT_V   = 6,
  ADC_ACCEL_Z  = 7,
};

// The following are not declared static so that they will be visible to adc.S.
volatile uint16_t samples_[ADC_SAMPLE_LENGTH][ADC_N_CHANNELS];
volatile uint8_t samples_index_ = 0;

static uint16_t baro_altitude_, battery_voltage_;
static int16_t acceleration_[3], angular_rate_[3];
static int16_t acc_offset_[3], gyro_offset_[3];


// =============================================================================
// Private function declarations:

static inline uint16_t ADCSample(enum ADCSensorIndex sensor);
static inline uint16_t SumRecords(enum ADCSensorIndex sensor);


// =============================================================================
// Accessors:

inline int16_t Acceleration(enum SensorAxes axis)
{
  return acceleration_[axis];  // x 5/12288 g
}

// -----------------------------------------------------------------------------
inline uint16_t Accelerometer(enum SensorAxes axis)
{
  // 5/1024 g/step
  switch (axis)
  {
    case X_AXIS:
      return ADCSample(ADC_ACCEL_X);
      break;
    case Y_AXIS:
      return ADCSample(ADC_ACCEL_Y);
      break;
    case Z_AXIS:
    default:
      return ADCSample(ADC_ACCEL_Z);
  }
}

// -----------------------------------------------------------------------------
inline int16_t AngularRate(enum SensorAxes axis)
{
  return angular_rate_[axis];  // x 625/9216 deg/sec
}

// -----------------------------------------------------------------------------
inline uint16_t BaroAltitude(void)
{
  return baro_altitude_;
}

// -----------------------------------------------------------------------------
inline uint16_t BatteryVoltage(void)
{
  return battery_voltage_;
}

// -----------------------------------------------------------------------------
inline uint16_t Gyro(enum SensorAxes axis)
{
  // 5/6.144 deg/s/step
  switch (axis)
  {
    case X_AXIS:
      return ADCSample(ADC_GYRO_X);
      break;
    case Y_AXIS:
      return ADCSample(ADC_GYRO_Y);
      break;
    case Z_AXIS:
    default:
      return ADCSample(ADC_GYRO_Z);
  }
}

// -----------------------------------------------------------------------------
inline uint16_t PressureSensor(void)
{
  return ADCSample(ADC_PRESSURE);
}


// =============================================================================
// Public functions:

// This function starts the ADC in free-running mode.
inline void ADCOn(void)
{
  ADCSRA = (1 << ADEN)  // ADC Enable
         | (1 << ADSC)  // ADC Start Conversion
         | (1 << ADATE)  // ADC Auto Trigger Enable
         | (1 << ADIF)  // (Clear the) ADC Interrupt Flag
         | (1 << ADIE)  // ADC Interrupt Enable
         | (1 << ADPS2)  // ADC Prescaler select bit 2
         | (1 << ADPS1)  // ADC Prescaler select bit 1
         | (1 << ADPS0);  // ADC Prescaler select bit 0
}

// -----------------------------------------------------------------------------
// This function immediately kills the ADC.
inline void ADCOff(void)
{
  ADCSRA = 0;  // Clear the ADC control register.
}

// -----------------------------------------------------------------------------
// This function sums several sensor readings (each reading the sample array) in
// order to increase fidelity.
void ProcessSensorReadings(void)
{
  acceleration_[X_AXIS] = -SumRecords(ADC_ACCEL_X) - acc_offset_[X_AXIS];
  acceleration_[Y_AXIS] = -SumRecords(ADC_ACCEL_Y) - acc_offset_[Y_AXIS];
  acceleration_[Z_AXIS] = SumRecords(ADC_ACCEL_Z) - acc_offset_[Z_AXIS];

  angular_rate_[X_AXIS] = -SumRecords(ADC_GYRO_X) - gyro_offset_[X_AXIS];
  angular_rate_[Y_AXIS] = -SumRecords(ADC_GYRO_Y) - gyro_offset_[Y_AXIS];
  angular_rate_[Z_AXIS] = -SumRecords(ADC_GYRO_Z) - gyro_offset_[Z_AXIS];

  baro_altitude_ = 1024 * ADC_SAMPLE_LENGTH - SumRecords(ADC_PRESSURE);

  battery_voltage_ = SumRecords(ADC_BATT_V);
}

// -----------------------------------------------------------------------------
// This function delays program execution until the ADC sample array has been
// fully refreshed at least once.
inline void WaitOneADCCycle(void)
{
  if (!(ADCSRA & _BV(ADSC))) return;  // ADC not running.
  while (!samples_index_) continue;
  while (!samples_index_) continue;
}


// =============================================================================
// Private functions:

// This function returns the most recent ADC sample for a particular sensor.
static inline uint16_t ADCSample(enum ADCSensorIndex sensor)
{
  return samples_[(samples_index_ - sensor) / ADC_N_CHANNELS][sensor];
}

// -----------------------------------------------------------------------------
// This function sums those samples in the sample array for a particular sensor.
static inline uint16_t SumRecords(enum ADCSensorIndex sensor)
{
  uint16_t result = 0;
  ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
    for (uint8_t i = 0; i < ADC_SAMPLE_LENGTH; i++)
      result += samples_[i][sensor];
  }
  return result;
}
