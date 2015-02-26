#ifndef TIMING_H_
#define TIMING_H_

#include <inttypes.h>
#include <util/atomic.h>


// ============================================================================+
// Global data:
//
// Warning: this data is made global so that it can be included in inline
// functions to speed execution of often-used functions. This global data should
// not be set outside of this program unit.

extern volatile int16_t g_ms_timestamp_;


// ============================================================================+
// Inline functions:

// This function returns a timestamp corresponding to "t" ms in the future. This
// timestamp can be checked against the current timestamp to see if a certain
// amount of time has passed. This function works for durations up to 32767 ms.
inline int16_t GetTimestampMillisFromNow(int16_t t)
{
  int16_t ms_timestamp;
  ATOMIC_BLOCK(ATOMIC_FORCEON) { ms_timestamp = g_ms_timestamp_; }
  return ms_timestamp + t + 1;
}

// -----------------------------------------------------------------------------
// This function compares a timestamp to the current timestamp and returns TRUE
// if the timestamp is in the past. This function works for durations up to
// 32767 ms.
inline uint8_t TimestampInPast(int16_t t)
{
  int16_t ms_timestamp;
  ATOMIC_BLOCK(ATOMIC_FORCEON) { ms_timestamp = g_ms_timestamp_; }
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
  ATOMIC_BLOCK(ATOMIC_FORCEON) { ms_timestamp = g_ms_timestamp_; }
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


// ============================================================================+
// Public functions:

void TimingInit(void);


#endif // TIMING_H_
