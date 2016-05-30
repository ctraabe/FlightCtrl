// The following 2nd order approximation of pressure altitude has been adopted:
//   h = 13305 - 182.09 pressure + 0.50192 pressure ^ 2

// Note that this expression is relative to standard-day sea-level pressure, so
// the results will vary depending on the weather conditions.

// The voltage output from the pressure sensor follows the following relation:
//   Version <= 2.1:
//     V_sensor = 0.045 * pressure (kPa) - 0.48
//   Version >= 2.5:
//     V_sensor = 0.027 * pressure (kPa) - 0.285

// Near sea level, pressure (kPa) is approximately -0.012 * altitude (m) + 100
// This leads to the relation:
//   Version <= 2.1:
//     V_sensor = 4.0 - 0.00054 * altitude (m)
//   Version >= 2.5:
//     V_sensor = 2.4 - 0.00032 * altitude (m)
// So, for a change in altitude of 1 m, the voltage output from
// the sensor only changes by 0.00054V.

// However, the microprocessor's A/D converter has a measurement range of 3
// Volts in steps of ~0.00293V (1024 bits). This implies a resolution worse than
// 5 meters per A/D converter LSB. To increase the resolution, a non-inverting
// operational amplifier is inserted between the pressure sensor and the A/D
// converter. This op-amp is wired to have a gain of 37.63 (Version <= 2.1) or
// 59.99 (version >= 2.5), which leads to a resolution of 0.14 meters per LSB
// (version <= 2.1) or 0.15 meters per LSB (version >= 2.5).

// By increasing the resolution, the measurable range becomes much smaller. At
// 0.14 m resolution, the 10-bit A/D converter can only capture a range of 140
// meters. In order to operate the MikroKopter at locations of various
// elevations (e.g. 5 meters in Odaiba vs. 600 meters at Mt. Takao), and to
// compensate for meteorological variations in barometric pressure, the
// measurable pressure range must be shifted so that current pressure lies
// within the measurable range.

// To accomplish this shift, the op-amp is wired such that the output can be
// biased by a pair of input voltages according to the following relation:
//   Version <= 2.1:
//     V_amplifier = 37.63 V_sensor - 5.2 V_fine - 10.4 V_coarse - 95.59V
//   Version >= 2.5:
//     V_amplifier = 60 V_sensor - 11.0 x (V_fine + V_course) - 66.0V

// Combined the above with the expression for V_sensor gives the relation for
// pressure from the voltage output from the amplifier and the bias voltages.
//   Version <= 2.1:
//     Pressure(kPa) = 0.59 V_amplifier + 3.1 V_fine + 6.1 V_coarse + 67
//   Version >= 2.5:
//     Pressure(kPa) = 0.62 V_amplifier + 6.8 (V_fine + V_coarse) + 41

// The biasing voltages, V_coarse and V_fine, are 0V to 5V biases driven by the
// output pins of TIMER0, OC0B and OC0A respectively. TIMER0 should be
// configured to drive these pins with a fast PWM. The PWMs are passed through a
// physical low-pass filter. The rise-time for this low-pass filter is on the
// order of 80 ms. OC0A and OC0B have 8-bit resolution.

// Considering the 10-bit, 3V ADC that reads V_amp and the 8-bit, 5V PWMs that
// produce the bias voltages gives the following relation:
//   Version <= 2.1:
//     Pressure(kPa) = ADC / 578 + 0.06 * OCR0A + 0.12 * OCR0B + 67
//   Version >= 2.5:
//     Pressure(kPa) = 0.0018 * ADC + 0.13 * (OCR0A + OCR0B) + 41

// Note: Given the structure of the amplifier, the measurable pressure altitude
// range for version <= 2.1 is limited to approximately -1,100 m to 3,400 m.

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

#ifdef V2_5
  #define ADC_SUM_TO_PRESSURE (0.0018 / (float)ADC_N_SAMPLES)
  #define COARSE_BIAS_STEPS_TO_PRESSURE_STEPS (-73 * ADC_N_SAMPLES);
  #define FINE_BIAS_STEPS_TO_PRESSURE_STEPS (-73 * ADC_N_SAMPLES);
#else
  #define ADC_SUM_TO_PRESSURE (1.0 / 578.0 / (float)ADC_N_SAMPLES)
  #define COARSE_BIAS_STEPS_TO_PRESSURE_STEPS (-69 * ADC_N_SAMPLES);
  #define FINE_BIAS_STEPS_TO_PRESSURE_STEPS (-35 * ADC_N_SAMPLES);
#endif
#define PRESSURE_TO_ALTITUDE_C2 (0.50192)
#define PRESSURE_TO_ALTITUDE_C1 (-182.09)
#define PRESSURE_TO_ALTITUDE_C0 (13305.0)
#define TIMER0_DIVIDER (1)

static uint8_t pressure_altitude_error_bits_ = 0x00;
static float delta_pressure_altitude_ = 0.0;
static float pressure_sum_to_altitude_ = -0.2;
static float pressure_0_ = 0.0, pressure_altitude_0_ = 0.0;
static int16_t biased_pressure_sum_0_ = 0;


// =============================================================================
// Accessors

uint8_t PressureAltitudeError(void)
{
  return pressure_altitude_error_bits_;
}

// -----------------------------------------------------------------------------
float DeltaPressureAltitude(void)
{
  return delta_pressure_altitude_;
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
      / COARSE_BIAS_STEPS_TO_PRESSURE_STEPS;
    if (adjustment == 0) break;
    bias_coarse += adjustment;

    // UARTTxByte('*');
    UARTPrintf("%i:%i", OCR0B, BiasedPressureSum());
    OCR0B = (uint8_t)S16Limit(bias_coarse, 0, 255);
  }

  // Save the found bias_coarse to EEPROM.
  eeprom_update_byte(&eeprom.pressure_coarse_bias, bias_coarse);

  // Search for the optimal fine bias
  for (uint8_t i = 0; i < 30; i++)
  {
    int16_t adjustment;
    adjustment = (kBaroAltThreeQuarterValue - (int16_t)BiasedPressureSum())
      / FINE_BIAS_STEPS_TO_PRESSURE_STEPS;
    if (adjustment == 0) break;
    bias_fine += adjustment;

    // UARTTxByte('.');
    UARTPrintf("%i:%i", OCR0A, BiasedPressureSum());
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
    pressure_altitude_error_bits_ &= ~PRESSURE_ERROR_BIT_BIAS_RANGE;
  }
  else
  {
    UARTPrintf("  ERROR: out of measurable range");
    pressure_altitude_error_bits_ |= PRESSURE_ERROR_BIT_BIAS_RANGE;
    return;
  }

  // Save the initial pressure sensor reading.
  biased_pressure_sum_0_ = (int16_t)BiasedPressureSum();

  // Compute the actual pressure corresponding to biased_pressure_sum_0_ given
  // the current bias settings.
  pressure_0_ = (float)biased_pressure_sum_0_ * ADC_SUM_TO_PRESSURE
    + (float)(2 * (OCR0B + 1) + OCR0A + 1) * 0.060 + 67.1;

  // Compute the pressure altitude corresponding to pressure_0_.
  pressure_altitude_0_ = PRESSURE_TO_ALTITUDE_C2 * pressure_0_ * pressure_0_
    + PRESSURE_TO_ALTITUDE_C1 * pressure_0_ + PRESSURE_TO_ALTITUDE_C0;

  // Compute the conversion factor from ADC pressure sum to pressure altitude.
  pressure_sum_to_altitude_ = (2.0 * PRESSURE_TO_ALTITUDE_C2 * pressure_0_
    + PRESSURE_TO_ALTITUDE_C1) * ADC_SUM_TO_PRESSURE;

  UARTPrintf("  current pressure = %f kPa", pressure_0_);
  UARTPrintf("  current pressure altitude = %f m", pressure_altitude_0_);
  UARTPrintf("  base reading = %i", biased_pressure_sum_0_);
  UARTPrintf("  conversion factor = %f", pressure_sum_to_altitude_);
}

// -----------------------------------------------------------------------------
void UpdatePressureAltitude(void)
{
  delta_pressure_altitude_ = ((int16_t)BiasedPressureSum()
    - biased_pressure_sum_0_) * pressure_sum_to_altitude_;
}
