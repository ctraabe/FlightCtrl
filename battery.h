#ifndef BATTERY_H_
#define BATTERY_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

void DetectBattery(void);

// -----------------------------------------------------------------------------
uint8_t BatteryLow(void);


#endif  // BATTERY_H_
