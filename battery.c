#include "battery.h"

#include "adc.h"
#include "buzzer.h"
#include "motors.h"
#include "timing.h"


// =============================================================================
// Private data:

static uint8_t voltage_low_limit_ = 231;
// static uint16_t current_estimate_ = 0, power_estimate_ = 0, used_capactiy_ = 0;
// static uint16_t update_timer_ = 0;


// =============================================================================
// Public functions:

void DetectBattery(void)
{
  if (ADCState() == ADC_INACTIVE) return;

  // TODO: if (motors_on) return

  WaitOneADCCycle();
  ProcessSensorReadings();

  uint8_t n_cells;
  const uint8_t kMinVoltsPerCell = 33;  // 3.3 V is the LiPo operational minimum
  const uint8_t kMaxVoltsPerCell = 43;  // 4.3 V is max for LiPo batteries
  for (n_cells = 2; (n_cells < 7) && (BatteryVoltage() > n_cells
    * kMaxVoltsPerCell); n_cells++) continue;

  voltage_low_limit_ = n_cells * kMinVoltsPerCell;
  BeepNTimes(n_cells, 200);
}

// -----------------------------------------------------------------------------
// This function estimates the current being drawn from the battery and
// calculates the corresponding reduction in battery capacity.
void BatteryUpdate(void)
{
}
