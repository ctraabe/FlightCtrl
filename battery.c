#include "battery.h"

#include "adc.h"
#include "buzzer.h"
#include "state.h"


// =============================================================================
// Private data:

static uint8_t voltage_low_limit_ = 0;


// =============================================================================
// Public functions:

// Detects the number of cells for the attached battery up to 4 cells (it is
// impossible to detect the number of cells for batteries with 5 or more cells).
void DetectBattery(void)
{
  if (MotorsRunning() || (ADCState() == ADC_INACTIVE)) return;

  WaitOneADCCycle();
  ProcessSensorReadings();

  uint8_t n_cells = 0;
  const uint8_t kMinVoltsPerCell = 33;  // 3.3 V is the LiPo operational minimum
  const uint8_t kMaxVoltsPerCell = 43;  // 4.3 V is max for LiPo batteries

  // Ignore battery checks if not being powered by a battery.
  if (BatteryVoltage() < kMinVoltsPerCell) return;

  while ((n_cells < 4) && (BatteryVoltage() > (n_cells * kMaxVoltsPerCell)))
    n_cells++;
  voltage_low_limit_ = n_cells * kMinVoltsPerCell;

  BeepNTimes(n_cells, 200);
  WaitForBuzzerToComplete();
}

// -----------------------------------------------------------------------------
uint8_t BatteryLow(void)
{
  return BatteryVoltage() < voltage_low_limit_;
}
