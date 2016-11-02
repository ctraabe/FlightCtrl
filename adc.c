#include "adc.h"

#include <stdlib.h>
#include <avr/io.h>
#include <util/atomic.h>

#include "custom_math.h"
#include "eeprom.h"
#include "main.h"
#include "mcu_pins.h"


// =============================================================================
// Private data:

#define ADC_MIDDLE_VALUE (1023 / 2)

// ADC sample indices
enum ADCSensorIndex {
  ADC_ACCEL_X  = 0,
  ADC_ACCEL_Y  = 1,
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

static float acceleration_[3], angular_rate_[3];
static uint16_t biased_pressure_sum_, battery_voltage_;
static int16_t accelerometer_sum_[3], gyro_sum_[3];
static int16_t acc_offset_[3];
static int16_t gyro_offset_[3] = { -ADC_MIDDLE_VALUE * ADC_N_SAMPLES,
  -ADC_MIDDLE_VALUE * ADC_N_SAMPLES, ADC_MIDDLE_VALUE * ADC_N_SAMPLES };


// =============================================================================
// Private function declarations:

static inline uint16_t ADCSample(enum ADCSensorIndex sensor);
static void CheckOffset(const int16_t offset[3], int16_t acceptable_deviation);
static inline uint16_t SumRecords(enum ADCSensorIndex sensor);


// =============================================================================
// Accessors:

// Body-axis acceleration from the accelerometer in g's.
float Acceleration(enum BodyAxes axis)
{
  return acceleration_[axis];
}

// -----------------------------------------------------------------------------
// Body-axis acceleration vector from accelerometer in g's.
const float * AccelerationVector(void)
{
  return acceleration_;
}

// -----------------------------------------------------------------------------
// Returns the most recent accelerometer reading. Scale is 5/1024 g/LSB.
uint16_t Accelerometer(enum BodyAxes axis)
{
  switch (axis)
  {
    case X_BODY_AXIS:
      if (BoardVersion() > 22) return ADCSample(ADC_ACCEL_X);
      return ADCSample(ADC_ACCEL_Y);
      break;
    case Y_BODY_AXIS:
      if (BoardVersion() > 22) return ADCSample(ADC_ACCEL_Y);
      return ADCSample(ADC_ACCEL_X);
      break;
    case Z_BODY_AXIS:
    default:
      return ADCSample(ADC_ACCEL_Z);
  }
}

// -----------------------------------------------------------------------------
// Returns the sum of the most recent ADC_N_SAMPLES accelerometer readings.
// Scale is 5/1024/ADC_N_SAMPLES g/LSB.
int16_t AccelerometerSum(enum BodyAxes axis)
{
  return accelerometer_sum_[axis];
}

// -----------------------------------------------------------------------------
enum ADCState ADCState(void)
{
  if (ADCSRA & _BV(ADSC)) return ADC_ACTIVE;
  else return ADC_INACTIVE;
}

// -----------------------------------------------------------------------------
// Body-axis angular rate from the gyros in rad/s.
float AngularRate(enum BodyAxes axis)
{
  return angular_rate_[axis];
}

// -----------------------------------------------------------------------------
// Body-axis angular rate vector from the gyros in rad/s.
const float * AngularRateVector(void)
{
  return angular_rate_;
}

// -----------------------------------------------------------------------------
// Latest measurement of battery voltage in 1/10 Volts.
uint16_t BatteryVoltage(void)
{
  return battery_voltage_;
}

// -----------------------------------------------------------------------------
// Returns the sum of the most recent ADC_N_SAMPLES biased pressure readings.
// Slope is 1/578/ADC_N_SAMPLES kPa/LSB and bias is determined by a pair of
// biasing voltages (see the PWM signals in pressure_altitude.c).
uint16_t BiasedPressureSum(void)
{
  return biased_pressure_sum_;
}

// -----------------------------------------------------------------------------
// Returns the most recent biased pressure reading. Slope is 1/578 kPa/LSB and
// bias is determined by a pair of biasing voltages (see the PWM signals in
// pressure_altitude.c).
uint16_t BiasedPressureSensor(void)
{
  return ADCSample(ADC_PRESSURE);
}

// -----------------------------------------------------------------------------
// Returns the most recent gyro reading. Scale is 5/6.144 deg/s/LSB.
uint16_t Gyro(enum BodyAxes axis)
{
  switch (axis)
  {
    case X_BODY_AXIS:
      return ADCSample(ADC_GYRO_X);
      break;
    case Y_BODY_AXIS:
      return ADCSample(ADC_GYRO_Y);
      break;
    case Z_BODY_AXIS:
    default:
      return ADCSample(ADC_GYRO_Z);
  }
}

// -----------------------------------------------------------------------------
// Returns the sum of the most recent ADC_N_SAMPLES gyro readings. Scale is
// 5/6.144/ADC_N_SAMPLES deg/s/LSB.
int16_t GyroSum(enum BodyAxes axis)
{
  return gyro_sum_[axis];
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

// -----------------------------------------------------------------------------
// This function loads the gyro offsets from EEPROM. Not totally necessary, but
//increases sanity of pre-initialized control computations.
void LoadGyroOffsets(void)
{
  eeprom_read_block((void*)gyro_offset_, (const void*)&eeprom.gyro_offset[0],
    sizeof(gyro_offset_));
}

// -----------------------------------------------------------------------------
// This function sums several sensor readings (each reading the sample array) in
// order to increase fidelity.
void ProcessSensorReadings(void)
{
  // Raw accelerometer reading minus bias.
  if (BoardVersion() > 22)
  {
    accelerometer_sum_[X_BODY_AXIS] = -SumRecords(ADC_ACCEL_X)
      - acc_offset_[X_BODY_AXIS];
    accelerometer_sum_[Y_BODY_AXIS] = -SumRecords(ADC_ACCEL_Y)
      - acc_offset_[Y_BODY_AXIS];
  }
  else
  {
    accelerometer_sum_[X_BODY_AXIS] = -SumRecords(ADC_ACCEL_Y)
      - acc_offset_[X_BODY_AXIS];
    accelerometer_sum_[Y_BODY_AXIS] = -SumRecords(ADC_ACCEL_X)
      - acc_offset_[Y_BODY_AXIS];
  }
  accelerometer_sum_[Z_BODY_AXIS] = -SumRecords(ADC_ACCEL_Z)
    - acc_offset_[Z_BODY_AXIS];

  // Convert raw accelerometer to g's.
  acceleration_[X_BODY_AXIS] = (float)accelerometer_sum_[X_BODY_AXIS]
    / ACCELEROMETER_SCALE / ADC_N_SAMPLES;
  acceleration_[Y_BODY_AXIS] = (float)accelerometer_sum_[Y_BODY_AXIS]
    / ACCELEROMETER_SCALE / ADC_N_SAMPLES;
  if (BoardVersion() > 21)
  {
    acceleration_[Z_BODY_AXIS] = (float)accelerometer_sum_[Z_BODY_AXIS]
      / ACCELEROMETER_2_2_SCALE / ADC_N_SAMPLES;
  }
  else
  {
    acceleration_[Z_BODY_AXIS] = (float)accelerometer_sum_[Z_BODY_AXIS]
      / ACCELEROMETER_SCALE / ADC_N_SAMPLES;
  }

  // Raw gyro reading minus bias.
  gyro_sum_[X_BODY_AXIS] = -SumRecords(ADC_GYRO_X) - gyro_offset_[X_BODY_AXIS];
  gyro_sum_[Y_BODY_AXIS] = -SumRecords(ADC_GYRO_Y) - gyro_offset_[Y_BODY_AXIS];
  gyro_sum_[Z_BODY_AXIS] = SumRecords(ADC_GYRO_Z) - gyro_offset_[Z_BODY_AXIS];

  // Convert raw gyro reading to rad/s.
  angular_rate_[X_BODY_AXIS] = (float)gyro_sum_[X_BODY_AXIS] / GYRO_SCALE
    / ADC_N_SAMPLES;
  angular_rate_[Y_BODY_AXIS] = (float)gyro_sum_[Y_BODY_AXIS] / GYRO_SCALE
    / ADC_N_SAMPLES;
  angular_rate_[Z_BODY_AXIS] = (float)gyro_sum_[Z_BODY_AXIS] / GYRO_SCALE
    / ADC_N_SAMPLES;

  // Raw pressure reading.
  biased_pressure_sum_ = SumRecords(ADC_PRESSURE);

  // The ADC records voltage in 31 steps per Volt. The following converts to the
  // desired 1 step per 0.1 V. The following gyration avoids overflow.
  battery_voltage_ = U16RoundRShiftU16(82 * U16RoundRShiftU16(SumRecords(
    ADC_BATT_V), ADC_N_SAMPLES_POW_OF_2 + 1) , 7);  //  1/10 Volts
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
// This function assumes that the vehicle is motionless (on the ground). It
// finds the average accelerometer readings over the period of 1 second
// (approximately) and considers the results to be the zero values of the
// accelerometers.
void ZeroAccelerometers(void)
{
  int32_t sample_sum[3] = { 0 };

  // TODO: Never block motor communication when running.
  // if (MotorsOn()) return 1;

  // Clear offsets.
  acc_offset_[X_BODY_AXIS] = 0;
  acc_offset_[Y_BODY_AXIS] = 0;
  acc_offset_[Z_BODY_AXIS] = 0;

  // Sum samples over about 1 second (2048 samples).
  const uint8_t kNSamplesPowOf2 = 11 - ADC_N_SAMPLES_POW_OF_2;
  const int32_t kNSamples = 1 << kNSamplesPowOf2;
  for (uint16_t i = 0; i < kNSamples; i++)
  {
    WaitOneADCCycle();
    ProcessSensorReadings();
    sample_sum[X_BODY_AXIS] += accelerometer_sum_[X_BODY_AXIS];
    sample_sum[Y_BODY_AXIS] += accelerometer_sum_[Y_BODY_AXIS];
    sample_sum[Z_BODY_AXIS] += accelerometer_sum_[Z_BODY_AXIS];
  }

  // Average the results and set as the offset.
  acc_offset_[X_BODY_AXIS] = S16RoundRShiftS32(sample_sum[X_BODY_AXIS],
    kNSamplesPowOf2);
  acc_offset_[Y_BODY_AXIS] = S16RoundRShiftS32(sample_sum[Y_BODY_AXIS],
    kNSamplesPowOf2);
  if (BoardVersion() > 21)
    acc_offset_[Z_BODY_AXIS] = S16RoundRShiftS32(sample_sum[Z_BODY_AXIS],
      kNSamplesPowOf2) + ADC_N_SAMPLES * ACCELEROMETER_2_2_SCALE;
  else
    acc_offset_[Z_BODY_AXIS] = S16RoundRShiftS32(sample_sum[Z_BODY_AXIS],
      kNSamplesPowOf2) + ADC_N_SAMPLES * ACCELEROMETER_SCALE;

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
// This function assumes that the vehicle is motionless on the ground. It finds
// the average gyro readings over the period of 1 second (approximately) and
// considers the results to be the zero values of the gyros.
void ZeroGyros(void)
{
  int32_t sample_sum[3] = { 0 };

  // TODO: Never block motor communication when running.
  // if (MotorsOn()) return 1;

  // Clear offsets.
  gyro_offset_[X_BODY_AXIS] = 0;
  gyro_offset_[Y_BODY_AXIS] = 0;
  gyro_offset_[Z_BODY_AXIS] = 0;

  // Sum samples over about 1 second (2000 samples).
  const uint8_t kNSamplesPowOf2 = 11 - ADC_N_SAMPLES_POW_OF_2;
  const int32_t kNSamples = 1 << kNSamplesPowOf2;
  for (uint16_t i = 0; i < kNSamples; i++)
  {
    WaitOneADCCycle();
    ProcessSensorReadings();
    sample_sum[X_BODY_AXIS] += gyro_sum_[X_BODY_AXIS];
    sample_sum[Y_BODY_AXIS] += gyro_sum_[Y_BODY_AXIS];
    sample_sum[Z_BODY_AXIS] += gyro_sum_[Z_BODY_AXIS];
  }

  // Average the results and set as the offset.
  gyro_offset_[X_BODY_AXIS] = S16RoundRShiftS32(sample_sum[X_BODY_AXIS],
    kNSamplesPowOf2);
  gyro_offset_[Y_BODY_AXIS] = S16RoundRShiftS32(sample_sum[Y_BODY_AXIS],
    kNSamplesPowOf2);
  gyro_offset_[Z_BODY_AXIS] = S16RoundRShiftS32(sample_sum[Z_BODY_AXIS],
    kNSamplesPowOf2);

  // TODO: Change these limits to something more reasonable.
  // Check that the zero values are within an acceptable range. The acceptable
  // range is specified in ADC steps from the ADC middle value (511).
  const uint16_t kAcceptableDeviation = 20;
  CheckOffset(gyro_offset_, kAcceptableDeviation);

  // Save the values in the EEPROM.
  // TODO: Make this contingent on success of CheckOffset
  eeprom_update_block((const void*)gyro_offset_, (void*)&eeprom.gyro_offset[0],
    sizeof(gyro_offset_));
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
static void CheckOffset(const int16_t offset[3], int16_t acceptable_deviation)
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
