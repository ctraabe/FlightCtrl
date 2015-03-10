// MikroKopter uses a pressure sensor to determine altitude by assuming the
// following relation between change in barometric pressure and change in
// altitude: pressure(kPa) = 100 - 0.012 * altitude(m). The voltage output from
// the pressure sensor follows the following relation: Vout = 0.045 *
// pressure(kPa) - 0.48. This leads to the relation: Vout = 4.5 - 0.00054 *
// altitude(m). So, for a change in altitude of 1m, the voltage output from the
// sensor only changes by 0.00054V.

// However, the microprocessor's A/D converter has a measurement range of 3
// Volts in steps of ~0.00293V (1024 bis). This implies a resolution worse than
// 5 meters per A/D converter step. To increase the resolution, a non-inverting
// operational amplifier is inserted between the pressure sensor and the A/D
// converter. This op-amp is wired to have a gain of 37.63, which leads to a
// resolution of 0.14 meters per step.

// By increasing the resolution, the measurable range becomes much smaller. At
// 0.14 m resolution, the 10-bit A/D converter can only capture a range of 140
// meters. In order to operate the Mikrokopter at locations of various
// elevations (e.g. 5 meters in Odaiba vs. 600 meters at Mt. Takao), and to
// compensate for meteorological variations in barometric pressure, the
// measurable pressure range must be shifted so that current pressure lies
// within the measurable range.

// To accomplish this shift, the op-amp is wired such that the output can be
// biased by a pair of input voltages according to the following relation:

// Vout = 37.63 Vsensor - 5.2 Vfine - 10.4 Vcoarse - 95.59V

// The biasing voltages, Vcoarse and Vfine, are driven by the output pins of
// TIMER0, OC0B and OC0A respectively. TIMER0 should be configured to drive
// these pins with a fast PWM. The PWMs are passed through a physical low-pass
// filter. The rise-time for this low-pass filter is on the order of 80 ms.

#include "pressure_altitude.h"

#include <avr/eeprom.h>
#include <avr/io.h>

#include "adc.h"
#include "mymath.h"
#include "timing.h"
#include "uart.h"


// ============================================================================+
// Private data:

#define TIMER0_DIVIDER (1)

uint8_t EEMEM eeprom_coarse_offset = 0;
uint16_t EEMEM eeprom_coarse_offset_steps_to_baro_altitude_steps
  = (uint16_t)-69 * ADC_N_SAMPLES;
uint16_t EEMEM eeprom_fine_offset_steps_to_baro_altitude_steps
  = (uint16_t)-35 * ADC_N_SAMPLES;


// ============================================================================+
// Public functions:

// TIMER0 is used to drive the PWM signals that set the coarse and fine offsets
// for the pressure sensor.
void PressureSensorInit(void)
{
  DDRB |= _BV(DDB4) | _BV(DDB3);  // Set pins PB4 (OC0B) PB3 (OC0A) to output

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
}

// -----------------------------------------------------------------------------
// This function adjusts the duty cycle of the PWM signals that are used to bias
// the output of the pressure sensor amplifier. This effectively shifts the 140
// m measurable range to encompass the starting altitude (see explanation at the
// top of this file).
void ResetPressureSensorRange(void) {
  const int16_t kBaroAltQuarterValue = 1024 / 4 * ADC_N_SAMPLES;

  // TODO: Never block communication to motors when running.
  // if (MotorsOn()) return;

  // Return if the ADC is not running.
  if (ADCState() != ADC_ACTIVE) return;

  // Initialize the fine adjustment to a middle value.
  int16_t offset_fine = 127;
  OCR0A = offset_fine;

  // Search for the optimal coarse offset.
  int16_t offset_coarse = 0;

  // Load the previously found coarse_offset and offset calibration from EEPROM.
  offset_coarse =  eeprom_read_byte(&eeprom_coarse_offset);
  int16_t coarse_offset_steps_to_baro_altitude_steps
     = eeprom_read_word(&eeprom_coarse_offset_steps_to_baro_altitude_steps);

  for (uint8_t i = 0; i < 30; i++)
  {
    UARTTxByte('*');
    OCR0B = (uint8_t)S16Limit(offset_coarse, 0, 255);
    Wait(250);
    ProcessSensorReadings();

    int16_t adjustment;
    adjustment = ((int16_t)BaroAltitude() - kBaroAltQuarterValue)
      / coarse_offset_steps_to_baro_altitude_steps;
    if (adjustment == 0) break;
    offset_coarse += adjustment;
  }

  // Save the found offset_coarse to EEPROM.
  eeprom_update_byte(&eeprom_coarse_offset, offset_coarse);

  // Load offset calibration from EEPROM.
  int16_t fine_offset_steps_to_baro_altitude_steps
     = eeprom_read_word(&eeprom_fine_offset_steps_to_baro_altitude_steps);

  // Search for the optimal fine offset
  for (uint8_t i = 0; i < 30; i++)
  {
    int16_t adjustment;
    adjustment = ((int16_t)BaroAltitude() - kBaroAltQuarterValue)
      / fine_offset_steps_to_baro_altitude_steps;
    if (adjustment == 0) break;
    offset_fine += adjustment;

    UARTTxByte('.');
    OCR0A = (uint8_t)S16Limit(offset_fine, 0, 255);
    Wait(250);
    ProcessSensorReadings();
  }

  // TODO: implement out of range error
  // if ((offset_fine < 10) || (offset_fine > 245)) error;
}

// -----------------------------------------------------------------------------
// This function attempts to calibrate the offset for more accurate shifts in
// the pressure altitude measurement range. Deviation from the theoretical
// relationship is possible due to imprecise resistor values.
void PressureSensorOffsetCalibration (void)
{
  // TODO: Never block communication to motors when running.
  // if (MotorsOn()) return;

  // Return if the ADC is not running.
  if (ADCState() != ADC_ACTIVE) return;

  // Make sure that the pressure sensor is reporting a value that is near a
  // quarter of its range.
  ResetPressureSensorRange();

  int16_t initial_reading;

  initial_reading = BaroAltitude();
  OCR0B += 1 << 3;  // 2^3 = 8. Adds approximately 1.6V to pressure sensor.
  Wait(250);
  eeprom_update_word(&eeprom_coarse_offset_steps_to_baro_altitude_steps,
    (uint16_t)S16RoundRShiftS16(BaroAltitude() - initial_reading, 3));
  OCR0B -= 1 << 3;
  Wait(250);

  initial_reading = BaroAltitude();
  OCR0A += 1 << 4;  // 2^4 = 16. Adds approximately 1.6V to pressure sensor.
  Wait(250);
  eeprom_update_word(&eeprom_fine_offset_steps_to_baro_altitude_steps,
    S16RoundRShiftS16(BaroAltitude() - initial_reading, 4));
  OCR0A -= 1 << 4;
  Wait(250);
}