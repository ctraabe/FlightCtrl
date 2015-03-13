#include "adc.h"

#include <stdlib.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <util/atomic.h>

#include "eeprom.h"


// =============================================================================
// Private data:

#define ADC_MIDDLE_VALUE (1023 / 2)

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
volatile uint16_t samples_[ADC_N_SAMPLES][ADC_N_CHANNELS];
volatile uint8_t samples_index_;

static uint16_t biased_pressure_, battery_voltage_;
static int16_t acceleration_[3], angular_rate_[3];
static int16_t acc_offset_[3];
static int16_t gyro_offset_[3] = { ADC_MIDDLE_VALUE * ADC_N_SAMPLES,
  ADC_MIDDLE_VALUE * ADC_N_SAMPLES, ADC_MIDDLE_VALUE * ADC_N_SAMPLES };


// =============================================================================
// Private function declarations:

static inline uint16_t ADCSample(enum ADCSensorIndex sensor);
static void CheckOffset(int16_t offset[3], int16_t acceptable_deviation);
static inline uint16_t SumRecords(enum ADCSensorIndex sensor);


// =============================================================================
// Accessors:

int16_t Acceleration(enum SensorAxes axis)
{
  return acceleration_[axis];  // x 5/12288 g
}

// -----------------------------------------------------------------------------
uint16_t Accelerometer(enum SensorAxes axis)
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
enum ADCState ADCState(void)
{
  if (ADCSRA & _BV(ADSC)) return ADC_ACTIVE;
  else return ADC_INACTIVE;
}

// -----------------------------------------------------------------------------
int16_t AngularRate(enum SensorAxes axis)
{
  return angular_rate_[axis];  // x 625/9216 deg/sec
}

// -----------------------------------------------------------------------------
uint16_t BatteryVoltage(void)
{
  return battery_voltage_;
}

// -----------------------------------------------------------------------------
uint16_t BiasedPressure(void)
{
  return biased_pressure_;
}

// -----------------------------------------------------------------------------
uint16_t BiasedPressureSensor(void)
{
  return ADCSample(ADC_PRESSURE);
}

// -----------------------------------------------------------------------------
uint16_t Gyro(enum SensorAxes axis)
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


// =============================================================================
// Public functions:

// This function starts the ADC in free-running mode.
void ADCOn(void)
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
void ADCOff(void)
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
  acceleration_[Z_AXIS] = -SumRecords(ADC_ACCEL_Z) - acc_offset_[Z_AXIS];

  angular_rate_[X_AXIS] = -SumRecords(ADC_GYRO_X) - gyro_offset_[X_AXIS];
  angular_rate_[Y_AXIS] = -SumRecords(ADC_GYRO_Y) - gyro_offset_[Y_AXIS];
  angular_rate_[Z_AXIS] = SumRecords(ADC_GYRO_Z) - gyro_offset_[Z_AXIS];

  biased_pressure_ = SumRecords(ADC_PRESSURE);

  battery_voltage_ = SumRecords(ADC_BATT_V);
}

// -----------------------------------------------------------------------------
// This function delays program execution until the ADC sample array has been
// fully refreshed.
void WaitOneADCCycle(void)
{
  if (!(ADCSRA & _BV(ADSC))) return;  // ADC not running.
  uint8_t samples_index_tmp = samples_index_;
  while (samples_index_tmp == samples_index_) continue;
  while (samples_index_tmp != samples_index_) continue;
}

// -----------------------------------------------------------------------------
// This function assumes that the Mikrokopter is motionless on the ground. It
// finds the average gyro readings over the period of 1 second (approximately)
// and considers the results to be the zero values of the gyros.
void ZeroGyros(void)
{
  uint32_t gyro_sum[3] = { 0 };

  // TODO: Never block motor communication when running.
  // if (MotorsOn()) return 1;

  // Clear offsets.
  gyro_offset_[X_AXIS] = 0;
  gyro_offset_[Y_AXIS] = 0;
  gyro_offset_[Z_AXIS] = 0;

  // Sum samples over about 1 second (2000 samples).
  // Note: the number of samples must be evenly divisible by ADC_N_SAMPLES.
  const uint16_t kNSamples = 2000 / ADC_N_SAMPLES;
  for (uint8_t i = 0; i < kNSamples; i++)
  {
    WaitOneADCCycle();
    ProcessSensorReadings();
    gyro_sum[X_AXIS] += (uint16_t)-angular_rate_[X_AXIS];
    gyro_sum[Y_AXIS] += (uint16_t)-angular_rate_[Y_AXIS];
    gyro_sum[Z_AXIS] += (uint16_t)angular_rate_[Z_AXIS];
  }

  // Average the results and set as the offset.
  gyro_offset_[X_AXIS] = (int16_t)(gyro_sum[X_AXIS] / kNSamples);
  gyro_offset_[Y_AXIS] = (int16_t)(gyro_sum[Y_AXIS] / kNSamples);
  gyro_offset_[Z_AXIS] = (int16_t)(gyro_sum[Z_AXIS] / kNSamples);

  // TODO: Change these limits to something more reasonable.
  // Check that the zero values are within an acceptable range. The acceptable
  // range is specified in ADC steps from the ADC middle value (511).
  const uint16_t kAcceptableDeviation = 20;
  CheckOffset(gyro_offset_, kAcceptableDeviation);
}

// -----------------------------------------------------------------------------
// This function assumes that the Mikrokopter is motionless (on the ground). It
// finds the average accelerometer readings over the period of 1 second
// (approximately) and considers the results to be the zero values of the
// accelerometers.
void ZeroAccelerometers(void)
{
  uint32_t acc_sum[3] = {0};

  // TODO: Never block motor communication when running.
  // if (MotorsOn()) return 1;

  // Clear offsets.
  acc_offset_[X_AXIS] = 0;
  acc_offset_[Y_AXIS] = 0;
  acc_offset_[Z_AXIS] = 0;

  // Sum samples over about 1 second (2000 samples).
  // Note: the number of samples must be evenly divisible by ADC_N_SAMPLES.
  const uint16_t kNSamples = 2000 / ADC_N_SAMPLES;
  for (uint8_t i = 0; i < kNSamples; i++)
  {
    WaitOneADCCycle();
    ProcessSensorReadings();
    acc_sum[X_AXIS] += (uint16_t)-acceleration_[X_AXIS];
    acc_sum[Y_AXIS] += (uint16_t)-acceleration_[Y_AXIS];
    acc_sum[Z_AXIS] += (uint16_t)-acceleration_[Z_AXIS];
  }

  // Average the results and set as the offset.
  acc_offset_[X_AXIS] = (int16_t)(acc_sum[X_AXIS] / kNSamples);
  acc_offset_[Y_AXIS] = (int16_t)(acc_sum[Y_AXIS] / kNSamples);
  acc_offset_[Z_AXIS] = (int16_t)(acc_sum[Z_AXIS] / kNSamples);

  // TODO: Change these limits to something more reasonable
  // Check that the zero values are within an acceptable range. The acceptable
  // range is specified in ADC steps from the ADC middle value (511).
  const uint16_t kAcceptableDeviation = 20;
  CheckOffset(acc_offset_, kAcceptableDeviation);

  // Save the values in the EEPROM.
  // TODO: Make this contingent on success of CheckOffset
  eeprom_update_block((const void*)acc_offset_, (void*)&eeprom.acc_offset[0],
    sizeof(acc_offset_));
}

// -----------------------------------------------------------------------------
// This function loads the accelerometer offsets from EEPROM so that
// accelerometers don't have to be re-calibrated every flight.
void LoadAccelerometerOffsets(void)
{
  eeprom_read_block((void*)acc_offset_, (const void*)&eeprom.acc_offset[0],
    sizeof(acc_offset_));

  // TODO: Change these limits to something more reasonable
  // Check that the zero values are within an acceptable range. The acceptable
  // range is specified in ADC steps from the ADC middle value (511).
  const uint16_t kAcceptableDeviation = 20;
  CheckOffset(acc_offset_, kAcceptableDeviation);
}


// =============================================================================
// Private functions:

// This function returns the most recent ADC sample for a particular sensor.
static inline uint16_t ADCSample(enum ADCSensorIndex sensor)
{
  return samples_[(samples_index_ - sensor) / ADC_N_CHANNELS][sensor];
}

// -----------------------------------------------------------------------------
// This function checks the calculated neutral value against predetermined
// limits and reports an error if a limit is exceeded.
static void CheckOffset(int16_t offset[3], int16_t acceptable_deviation)
{
  for (int i = 0; i < 3; i++)
  {
    int16_t deviation = abs(offset[i] / ADC_N_SAMPLES - ADC_MIDDLE_VALUE);
    if (deviation > acceptable_deviation)
      // TODO: set an error
      return;
  }
}

// -----------------------------------------------------------------------------
// This function sums those samples in the sample array for a particular sensor.
static inline uint16_t SumRecords(enum ADCSensorIndex sensor)
{
  uint16_t result = 0;
  ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
    for (uint8_t i = 0; i < ADC_N_SAMPLES; i++)
      result += samples_[i][sensor];
  }
  return result;
}
