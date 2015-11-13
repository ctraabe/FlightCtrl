// MikroKopter uses a pressure sensor to determine altitude by assuming the
// following relation between change in barometric pressure and change in
// altitude: pressure(kPa) = 100 - 0.012 * altitude (m). The voltage output from
// the pressure sensor follows the following relation: V_sensor = 0.045 *
// pressure (kPa) - 0.48. This leads to the relation: V_sensor = 4.5 - 0.00054 *
// altitude (m). So, for a change in altitude of 1 m, the voltage output from
// the sensor only changes by 0.00054V.

// However, the microprocessor's A/D converter has a measurement range of 3
// Volts in steps of ~0.00293V (1024 bis). This implies a resolution worse than
// 5 meters per A/D converter step. To increase the resolution, a non-inverting
// operational amplifier is inserted between the pressure sensor and the A/D
// converter. This op-amp is wired to have a gain of 37.63, which leads to a
// resolution of 0.14 meters per step.

// By increasing the resolution, the measurable range becomes much smaller. At
// 0.14 m resolution, the 10-bit A/D converter can only capture a range of 140
// meters. In order to operate the MikroKopter at locations of various
// elevations (e.g. 5 meters in Odaiba vs. 600 meters at Mt. Takao), and to
// compensate for meteorological variations in barometric pressure, the
// measurable pressure range must be shifted so that current pressure lies
// within the measurable range.

// To accomplish this shift, the op-amp is wired such that the output can be
// biased by a pair of input voltages according to the following relation:

// V_amplifier = 37.63 V_sensor - 5.2 V_fine - 10.4 V_coarse - 95.59V

// Combined the above with the expression for V_sensor gives the relation for
// pressure from the voltage output from the amplifier and the bias voltages.

// Pressure(kPa) = 0.59 V_amplifier + 3.1 V_fine + 6.1 V_coarse + 67

// The biasing voltages, V_coarse and V_fine, are driven by the output pins of
// TIMER0, OC0B and OC0A respectively. TIMER0 should be configured to drive
// these pins with a fast PWM. The PWMs are passed through a physical low-pass
// filter. The rise-time for this low-pass filter is on the order of 80 ms.

// Considering the 10-bit, 3 V ADC that reads V_amp and the 8-bit, 5 V PWMs that
// produce the bias voltages gives the following relation:

// Pressure(kPa) = ADC / 578 + 0.06 * V_fine + 0.12 * V_coarse + 67

// Note: Given the structure of the amplifier, the measurable pressure altitude
// range is limited to between -1,100 m and 3,400 m approximately.

// The following 2nd order approximation of pressure altitude has been adopted:

// h = 13305 - 182.09 pressure + 0.50192 pressure ^ 2

// Note that this expression is relative to standard-day sea-level pressure, so
// the results will vary depending on the weather conditions.

#include "pressure_altitude.h"

#include <stdlib.h>

#include "adc.h"
#include "custom_math.h"
#include "eeprom.h"
#include "mcu_pins.h"
#include "timing.h"
#include "uart.h"


// =============================================================================
// Private data:

#define TIMER0_DIVIDER (1)

static uint8_t pressure_altitude_error_bits_ = 0x00;
static int16_t coarse_bias_steps_to_pressure_steps_ = 0;
static int16_t fine_bias_steps_to_pressure_steps_ = 0;
static float pressure_ = 0.0, pressure_altitude_ = 0.0;


// =============================================================================
// Private function declarations:

static void LoadPressureSensorBiasCalibration(void);
static void CheckPressureSensorBiasCalibration(void);


// =============================================================================
// Accessors

uint8_t PressureAltitudeError(void)
{
  return pressure_altitude_error_bits_;
}


// =============================================================================
// Public functions:

// TIMER0 is used to drive the PWM signals that set the coarse and fine biases
// for the pressure sensor.
void PressureSensorInit(void)
{
  // Set bias pins to output.
  PRESSURE_BIAS_DDR |= PRESSURE_BIAS_COARSE_PIN | PRESSURE_BIAS_FINE_PIN;

  // Waveform generation mode bits: Fast PWM
  TCCR0B = (0 << WGM02);
  TCCR0A = (1 << WGM01)
         | (1 << WGM00)
  // Compare match output A mode bits: Set OC0A at BOTTOM clear on match
         | (1 << COM0A1)
         | (0 << COM0A0)
  // Compare match output B mode bits: Set OC0B at BOTTOM clear on match
         | (1 << COM0B1)
         | (0 << COM0B0);
  // Clock select bits:
  switch (TIMER0_DIVIDER)
  {
    case 1:
      TCCR0B |= (0 << CS02) | (0 << CS01) | (1 << CS00);
      break;
    case 8:
      TCCR0B |= (0 << CS02) | (1 << CS01) | (0 << CS00);
      break;
    case 64:
      TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);
      break;
    case 256:
      TCCR0B |= (1 << CS02) | (0 << CS01) | (0 << CS00);
      break;
    case 1024:
      TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00);
      break;
    case 0:
    default:
      TCCR0B |= (0 << CS02) | (0 << CS01) | (0 << CS00);
      break;
  }
  OCR0B = eeprom_read_byte(&eeprom.pressure_coarse_bias);

  LoadPressureSensorBiasCalibration();
}

// -----------------------------------------------------------------------------
// This function adjusts the duty cycle of the PWM signals that are used to bias
// the output of the pressure sensor amplifier. This effectively shifts the 140
// m measurable range to encompass the starting altitude (see explanation at the
// top of this file).
void ResetPressureSensorRange(void)
{
  const int16_t kBaroAltThreeQuarterValue = 3 * 1024 / 4 * ADC_N_SAMPLES;

  // TODO: Never block communication to motors when running.
  // if (MotorsOn()) return;

  // Return if the ADC is not running.
  if (ADCState() != ADC_ACTIVE) return;

  UARTPrintf("pressure_altitude: setting measurement range:");
  UARTTxByte('|');

  // Initialize the fine adjustment to a middle value.
  int16_t bias_fine = 127;
  OCR0A = bias_fine;

  // Search for the optimal coarse bias.
  int16_t bias_coarse = OCR0B;

  for (uint8_t i = 0; i < 30; i++)
  {
    Wait(300);
    ProcessSensorReadings();

    int16_t adjustment;
    adjustment = (kBaroAltThreeQuarterValue - (int16_t)BiasedPressureSum())
      / coarse_bias_steps_to_pressure_steps_;
    if (adjustment == 0) break;
    bias_coarse += adjustment;

    UARTTxByte('*');
    OCR0B = (uint8_t)S16Limit(bias_coarse, 0, 255);
  }

  // Save the found bias_coarse to EEPROM.
  eeprom_update_byte(&eeprom.pressure_coarse_bias, bias_coarse);

  // Search for the optimal fine bias
  for (uint8_t i = 0; i < 30; i++)
  {
    int16_t adjustment;
    adjustment = (kBaroAltThreeQuarterValue - (int16_t)BiasedPressureSum())
      / fine_bias_steps_to_pressure_steps_;
    if (adjustment == 0) break;
    bias_fine += adjustment;

    UARTTxByte('.');
    OCR0A = (uint8_t)S16Limit(bias_fine, 0, 255);
    Wait(300);
    ProcessSensorReadings();
  }
  UARTPrintf("|");  // New line

  // TODO: Perhaps make this more restrictive
  if ((bias_fine > 10) || (bias_fine < 245))
  {
    UARTPrintf("  coarse bias set to %u", bias_coarse);
    UARTPrintf("  fine bias set to %u", bias_fine);
    UpdatePressureAltitude();
    UARTPrintf("  current pressure = %f kPa", pressure_);
    UARTPrintf("  current pressure altitude = %f m", pressure_altitude_);
    pressure_altitude_error_bits_ &= ~PRESSURE_ERROR_BIT_BIAS_RANGE;
  }
  else
  {
    UARTPrintf("  ERROR: out of measurable range");
    pressure_altitude_error_bits_ |= PRESSURE_ERROR_BIT_BIAS_RANGE;
  }
}

// -----------------------------------------------------------------------------
// This function attempts to calibrate the bias for more accurate shifts in
// the pressure altitude measurement range. Deviation from the theoretical
// relationship is possible due to imprecise resistor values.
void PressureSensorBiasCalibration(void)
{
  // TODO: Never block communication to motors when running.
  // if (MotorsOn()) return;

  // Return if the ADC is not running.
  if (ADCState() != ADC_ACTIVE) return;

  // Make sure that the pressure sensor is reporting a value that is near three
  // quarters of its range.
  ResetPressureSensorRange();

  int16_t initial_reading;

  initial_reading = BiasedPressureSum();
  OCR0B += 1 << 3;  // 2^3 = 8. Subtracts approximately 1.6V to pressure sensor.
  Wait(300);
  ProcessSensorReadings();
  coarse_bias_steps_to_pressure_steps_ = S16RoundRShiftS16(BiasedPressureSum()
    - initial_reading, 3);

  OCR0B -= 1 << 3;
  Wait(300);
  ProcessSensorReadings();

  initial_reading = BiasedPressureSum();
  OCR0A += 1 << 4;  // 2^4 = 16. Subtracts approximately 1.6V to pressure sensor.
  Wait(300);
  ProcessSensorReadings();
  fine_bias_steps_to_pressure_steps_ = S16RoundRShiftS16(BiasedPressureSum()
    - initial_reading, 4);

  OCR0A -= 1 << 4;

  CheckPressureSensorBiasCalibration();

  if (~pressure_altitude_error_bits_ & PRESSURE_ERROR_BIT_COARSE_CALIBRATION)
    eeprom_update_word(&eeprom.coarse_bias_steps_to_pressure_steps,
      (uint16_t)coarse_bias_steps_to_pressure_steps_);

  if (~pressure_altitude_error_bits_ & PRESSURE_ERROR_BIT_FINE_CALIBRATION)
    eeprom_update_word(&eeprom.fine_bias_steps_to_pressure_steps,
      (uint16_t)fine_bias_steps_to_pressure_steps_);
}

// -----------------------------------------------------------------------------
void UpdatePressureAltitude(void)
{
  pressure_ = (float)BiasedPressureSum() / (float)(578 * ADC_N_SAMPLES)
    + (float)(OCR0B + 1) * 0.12 + (float)(OCR0A + 1) * 0.060 + 67.1;

  pressure_altitude_ = 13305.0 - 182.09 * pressure_ + 0.50192
    * square(pressure_);
}


// =============================================================================
// Private functions:

// This function loads the pressure sensor bias calibration from EEPROM so that
// it doesn't have to be re-calibrated every flight.
static void LoadPressureSensorBiasCalibration(void)
{
  coarse_bias_steps_to_pressure_steps_ = (int16_t)eeprom_read_word(
    &eeprom.coarse_bias_steps_to_pressure_steps);
  fine_bias_steps_to_pressure_steps_ = (int16_t)eeprom_read_word(
    &eeprom.fine_bias_steps_to_pressure_steps);

  CheckPressureSensorBiasCalibration();

  if (pressure_altitude_error_bits_ & PRESSURE_ERROR_BIT_COARSE_CALIBRATION)
    eeprom_update_word(&eeprom.coarse_bias_steps_to_pressure_steps,
      (uint16_t)(-69 * ADC_N_SAMPLES));

  if (pressure_altitude_error_bits_ & PRESSURE_ERROR_BIT_FINE_CALIBRATION)
    eeprom_update_word(&eeprom.fine_bias_steps_to_pressure_steps,
      (uint16_t)(-35 * ADC_N_SAMPLES));
}

// -----------------------------------------------------------------------------
static void CheckPressureSensorBiasCalibration(void)
{
  const int16_t kExpectedCoarseStepsToPressureSteps = -69 * ADC_N_SAMPLES;
  const int16_t kExpectedFineStepsToPressureSteps = -35 * ADC_N_SAMPLES;
  const int16_t kAcceptableDeviationPercent = 10;

  int16_t coarse_bias_deviation = abs(coarse_bias_steps_to_pressure_steps_
    - kExpectedCoarseStepsToPressureSteps);

  if (coarse_bias_deviation < (abs(kExpectedCoarseStepsToPressureSteps)
    * kAcceptableDeviationPercent) / 100)
  {
    pressure_altitude_error_bits_ &= ~PRESSURE_ERROR_BIT_COARSE_CALIBRATION;
    UARTPrintf("pressure_altitude: a coarse bias step gives %i pressure steps",
      coarse_bias_steps_to_pressure_steps_);
  }
  else
  {
    pressure_altitude_error_bits_ |= PRESSURE_ERROR_BIT_COARSE_CALIBRATION;
    coarse_bias_steps_to_pressure_steps_ = kExpectedCoarseStepsToPressureSteps;
    UARTPrintf("pressure_altitude: ERROR: coarse bias calibration");
  }

  int16_t fine_bias_deviation = abs(fine_bias_steps_to_pressure_steps_
    - kExpectedFineStepsToPressureSteps);

  if (fine_bias_deviation < (abs(kExpectedFineStepsToPressureSteps)
    * kAcceptableDeviationPercent) / 100)
  {
    pressure_altitude_error_bits_ &= ~PRESSURE_ERROR_BIT_FINE_CALIBRATION;
    UARTPrintf("pressure_altitude: a fine bias step gives %i pressure steps",
      fine_bias_steps_to_pressure_steps_);
  }
  else
  {
    pressure_altitude_error_bits_ |= PRESSURE_ERROR_BIT_FINE_CALIBRATION;
    fine_bias_steps_to_pressure_steps_ = kExpectedFineStepsToPressureSteps;
    UARTPrintf("pressure_altitude: ERROR: fine bias calibration");
  }
}

