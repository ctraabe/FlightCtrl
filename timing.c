#include "timing.h"

#include <avr/io.h>


// ============================================================================+
// Global data:

volatile int16_t g_ms_timestamp_ = 0;


// ============================================================================+
// Private data:

#define TIMER1_DIVIDER 1
#define TIMER3_DIVIDER 8
#define F_ICR1 1000
#define F_ICR3 128


// ============================================================================+
// Public functions:

// This function initializes TIMER3. This timer triggers the interrupt
// "TIMER3_COMPA" at 1kHz.
void TimingInit(void)
{
  // Clear TIMER1 registers.
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;
  TIMSK1 = 0;
  // Waveform generation mode bits:
  TCCR1B |= (1 << WGM13);
  TCCR1B |= (1 << WGM12);
  TCCR1A |= (0 << WGM11);
  TCCR1A |= (0 << WGM10);
  // Compare match output A mode bits:
  TCCR1A |= (0 << COM1A1);
  TCCR1A |= (0 << COM1A0);
  // Compare match output B mode bits:
  TCCR1A |= (0 << COM1B1);
  TCCR1A |= (0 << COM1B0);
  // Force output compare A bit:
  TCCR1B |= (0 << FOC1A);
  // Force output compare B bit:
  TCCR1B |= (0 << FOC1B);
  // Clock select bits:
  switch (TIMER1_DIVIDER)
  {
    case 1:
      TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
      break;
    case 8:
      TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
      break;
    case 64:
      TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
      break;
    case 256:
      TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
      break;
    case 1024:
      TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
      break;
    case 0:
    default:
      TCCR1B |= (0 << CS12) | (0 << CS11) | (0 << CS10);
      break;
  }
  // Overflow interrupt enable bit:
  TIMSK1 |= (0 << TOIE1);
  // Input capture register (or TOP in CTC with WGM13 set):
  TIMSK1 |= (1 << ICIE1);  // Input capture interrupt enable (or TOP in CTC).
  ICR1 = F_CPU / TIMER1_DIVIDER / F_ICR1 - 1;  // = 19999
  // Output compare match A
  TIMSK1 |= (0 << OCIE1A);  // Output compare match A interrupt enable.
  OCR1A = 0;
  // Output compare match B
  TIMSK1 |= (0 << OCIE1B);  // Output compare match B interrupt enable.
  OCR1B = 0;
  // Clear the timer.
  TCNT1 = 0;

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
      TCCR3B |= (0 << CS32) | (0 << CS31) | (1 << CS30);
      break;
    case 8:
      TCCR3B |= (0 << CS32) | (1 << CS31) | (0 << CS30);
      break;
    case 64:
      TCCR3B |= (0 << CS32) | (1 << CS31) | (1 << CS30);
      break;
    case 256:
      TCCR3B |= (1 << CS32) | (0 << CS31) | (0 << CS30);
      break;
    case 1024:
      TCCR3B |= (1 << CS32) | (0 << CS31) | (1 << CS30);
      break;
    case 0:
    default:
      TCCR3B |= (0 << CS32) | (0 << CS31) | (0 << CS30);
      break;
  }
  // Overflow interrupt enable bit:
  TIMSK3 |= (0 << TOIE3);
  // Input capture register (or TOP in CTC with WGM33 set):
  TIMSK3 |= (1 << ICIE3);  // Input capture interrupt enable (or TOP in CTC).
  ICR3 = F_CPU / TIMER3_DIVIDER / F_ICR3 - 1;  // = 19530.25 (~13 PPM error)
  // Output compare match A
  TIMSK3 |= (0 << OCIE3A);  // Output compare match A interrupt enable.
  OCR3A = 0;
  // Output compare match B
  TIMSK3 |= (0 << OCIE3B);  // Output compare match B interrupt enable.
  OCR3B = 0;
  // Clear the timer.
  TCNT3 = 0;
}
