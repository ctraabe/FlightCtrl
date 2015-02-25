#include "timing.h"

#include <avr/io.h>


// ============================================================================+
// Private data:

volatile uint16_t ms_timestamp_ = 0;


// ============================================================================+
// Public functions:

// This function initializes TIMER3. This timer triggers the interrupt
// "TIMER3_COMPA" at 1kHz.
void TimingInit(void)
{
  // Clear TIMER3 registers.
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3C = 0;
  TIMSK3 = 0;
  // Waveform generation mode bits:
  TCCR3B |= (1 << WGM33);
  TCCR3B |= (1 << WGM32);
  TCCR3A |= (0 << WGM31);
  TCCR3A |= (0 << WGM30);
  // Compare match output A mode bits:
  TCCR3A |= (0 << COM3A1);
  TCCR3A |= (0 << COM3A0);
  // Compare match output B mode bits:
  TCCR3A |= (0 << COM3B1);
  TCCR3A |= (0 << COM3B0);
  // Force output compare A bit:
  TCCR3B |= (0 << FOC3A);
  // Force output compare B bit:
  TCCR3B |= (0 << FOC3B);
  // Clock select bits:
  switch (TIMER3_DIVIDER)
  {
    case 1:
      TCCR3B |= 0<<CS32 | 0<<CS31 | 1<<CS30;
      break;
    case 8:
      TCCR3B |= 0<<CS32 | 1<<CS31 | 0<<CS30;
      break;
    case 64:
      TCCR3B |= 0<<CS32 | 1<<CS31 | 1<<CS30;
      break;
    case 256:
      TCCR3B |= 1<<CS32 | 0<<CS31 | 0<<CS30;
      break;
    case 1024:
      TCCR3B |= 1<<CS32 | 0<<CS31 | 1<<CS30;
      break;
    case 0:
    default:
      TCCR3B |= 0<<CS32 | 0<<CS31 | 0<<CS30;
      break;
  }
  // Overflow interrupt enable bit:
  TIMSK3 |= (0 << TOIE3);
  // Input capture register (or TOP in CTC with WGM33 set to 1):
  ICR3 = F_CPU / TIMER3_DIVIDER / F_ICR3 - 1;  // (~13 PPM error)
  // Output compare match A
  TIMSK3 |= (1 << OCIE3A);  // Output compare match A interrupt enable.
  OCR3A = F_CPU / TIMER3_DIVIDER / F_OCR3A;  // = 2500;
  // Output compare match B
  TIMSK3 |= (0 << OCIE3B);  // Output compare match B interrupt enable.
  OCR3BH = 0;
  OCR3BL = 0;
  // Clear the timer.
  TCNT3H = 0;
  TCNT3L = 0;
}

// -----------------------------------------------------------------------------
// This function delays execution of the program for "t" ms. Functions triggered
// by interrupts will still execute during this period. This function works for
// time periods up to 32767 ms.
void Wait(uint16_t w)
{
  uint16_t timestamp = GetTimestampMillisFromNow(w);
  while (!TimestampInPast(timestamp));
}
