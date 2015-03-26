#ifndef BATTERY_H_
#define BATTERY_H_

#include <inttypes.h>


// =============================================================================
// Public functions:

void DetectBattery(void);

// -----------------------------------------------------------------------------
// This function estimates the current being drawn from the battery and
// calculates the corresponding reduction in battery capacity.
void BatteryUpdate(void);


#endif  // BATTERY_H_
