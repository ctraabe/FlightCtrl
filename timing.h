#ifndef TIMING_H_
#define TIMING_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

// This function initializes TIMER1 and TIMER3. This timer trigger interrupts
// at 1 kHz and 128 Hz respectively.
void TimingInit(void);

// -----------------------------------------------------------------------------
// This function returns the current timestamp.
int16_t GetTimestamp(void);

// -----------------------------------------------------------------------------
// This function returns a timestamp corresponding to "t" ms in the future. This
// timestamp can be checked against the current timestamp to see if a certain
// amount of time has passed. This function works for durations up to 32767 ms.
int16_t GetTimestampMillisFromNow(int16_t t);

// -----------------------------------------------------------------------------
// This function compares a timestamp to the current timestamp and returns TRUE
// if the timestamp is in the past. This function works for durations up to
// 32767 ms.
uint8_t TimestampInPast(int16_t t);

// -----------------------------------------------------------------------------
// This function returns the amount of time that has elapsed since the timestamp
// "last_time" has occurred. This function works for time periods up to 65535
// ms. The function also automatically updates last_time so that it can be
// easily be called periodically.
uint16_t MillisSinceTimestamp(int16_t *last_time);

// -----------------------------------------------------------------------------
// This function delays execution of the program for "t" ms. Functions triggered
// by interrupts will still execute during this period. This function works for
// time periods up to 32767 ms.
void Wait(uint16_t w);

// -----------------------------------------------------------------------------
// This function returns the value of TIMER1, which increments with every CPU
// clock cycle, which can be used to measure execution time.
int16_t GetCycleStamp(void);

// -----------------------------------------------------------------------------
// This function returns the amount of time that has elapsed since the timestamp
// "last_time" has occurred. This function works for time periods up to 65535
// ms. The function also automatically updates last_time so that it can be
// easily be called periodically.
uint16_t CyclesSince(int16_t cycle);


#endif  // TIMING_H_
