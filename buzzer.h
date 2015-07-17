#ifndef BUZZER_H_
#define BUZZER_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

void BuzzerInit(void);

// -----------------------------------------------------------------------------
// This function commands the buzzer to sound for the requested duration in
// milliseconds (up to 2 seconds).
void BeepDuration(uint16_t duration);

// -----------------------------------------------------------------------------
// This function commands the buzzer to sound n times. Each repetition will
// sound for the requested duration, followed by a silence of the same duration.
// The maximum allowable duration is 1 second.
void BeepNTimes(uint8_t n, uint16_t duration);

// -----------------------------------------------------------------------------
// This function commands the buzzer to beep in the specified pattern. (Each 1
// in the pattern represents a beep duration of 1/16 seconds. Leading zeros
// will be ignored, other zeros will be 1/16 seconds of silence.)
void BeepPattern(uint32_t beep_pattern);

// -----------------------------------------------------------------------------
// This function updates the status of the buzzer. It should be called at 16Hz.
void UpdateBuzzer(void);

// -----------------------------------------------------------------------------
void WaitForBuzzerToComplete(void);


#endif  // BUZZER_H_
