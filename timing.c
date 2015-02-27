#include "timing.h"

#include <avr/io.h>
#include <util/atomic.h>


// =============================================================================
// Private data:

#define TIMER1_DIVIDER 1
#define TIMER3_DIVIDER 8
#define F_ICR1 1000
#define F_ICR3 128

// The following is not declared static so that it will be visible to timing.S.
volatile int16_t ms_timestamp_;


// =============================================================================
// Public functions:

// This function initializes TIMER1 and TIMER3. This timer trigger interrupts
// at 1 kHz and 128 Hz respectively.
void TimingInit(void)
{
  // Waveform generation mode bits:
  TCCR1B = (1 << WGM13)
         | (1 << WGM12);
  TCCR1A = (0 << WGM11)
         | (0 << WGM10);
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
  // Input capture register (or TOP in CTC with WGM13 set):
  TIMSK1 = (1 << ICIE1);  // Input capture interrupt enable (or TOP in CTC).
  ICR1 = F_CPU / TIMER1_DIVIDER / F_ICR1 - 1;  // = 19999

  // Waveform generation mode bits:
  TCCR3B = (1 << WGM33)
         | (1 << WGM32);
  TCCR3A = (0 << WGM31)
         | (0 << WGM30);
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
  // Input capture register (or TOP in CTC with WGM33 set):
  TIMSK3 = (1 << ICIE3);  // Input capture interrupt enable (or TOP in CTC).
  ICR3 = F_CPU / TIMER3_DIVIDER / F_ICR3 - 1;  // = 19530.25 (~13 PPM error)
}

// -----------------------------------------------------------------------------
// This function returns a timestamp corresponding to "t" ms in the future. This
// timestamp can be checked against the current timestamp to see if a certain
// amount of time has passed. This function works for durations up to 32767 ms.
inline int16_t GetTimestampMillisFromNow(int16_t t)
{
  int16_t ms_timestamp;
  ATOMIC_BLOCK(ATOMIC_FORCEON) { ms_timestamp = ms_timestamp_; }
  return ms_timestamp + t + 1;
}

// -----------------------------------------------------------------------------
// This function compares a timestamp to the current timestamp and returns TRUE
// if the timestamp is in the past. This function works for durations up to
// 32767 ms.
inline uint8_t TimestampInPast(int16_t t)
{
  int16_t ms_timestamp;
  ATOMIC_BLOCK(ATOMIC_FORCEON) { ms_timestamp = ms_timestamp_; }
  return (t - ms_timestamp) < 0;
}

// -----------------------------------------------------------------------------
// This function returns the amount of time that has elapsed since the timestamp
// "last_time" has occurred. This function works for time periods up to 65535
// ms. The function also automatically updates last_time so that it can be
// easily be called periodically.
inline uint16_t MillisSinceTimestamp(int16_t *last_time)
{
  int16_t ms_timestamp;
  ATOMIC_BLOCK(ATOMIC_FORCEON) { ms_timestamp = ms_timestamp_; }
  uint16_t ret = (uint16_t)(ms_timestamp - *last_time);
  *last_time = ms_timestamp;
  return ret;
}

// -----------------------------------------------------------------------------
// This function delays execution of the program for "t" ms. Functions triggered
// by interrupts will still execute during this period. This function works for
// time periods up to 32767 ms.
inline void Wait(uint16_t w)
{
  int16_t timestamp = GetTimestampMillisFromNow(w);
  while (!TimestampInPast(timestamp));
}

// -----------------------------------------------------------------------------
// This function returns the value of TIMER1, which increments with every CPU
// clock cycle, which can be used to measure execution time.
inline int16_t GetCycleStamp(void)
{
  return (int16_t)TCNT1;
}

// -----------------------------------------------------------------------------
// This function returns the amount of time that has elapsed since the timestamp
// "last_time" has occurred. This function works for time periods up to 65535
// ms. The function also automatically updates last_time so that it can be
// easily be called periodically.
inline uint16_t CyclesSince(int16_t cycle)
{
  return TCNT1 - cycle - 4;  // Minus 4 cycles for loading the counter value.
}
