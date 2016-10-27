#include "battery.h"

#include "adc.h"
#include "buzzer.h"
#include "motors.h"


// =============================================================================
// Private data:

static uint8_t voltage_low_limit_ = 255;
// static uint16_t current_estimate_ = 0, power_estimate_ = 0, used_capactiy_ = 0;
// static uint16_t update_timer_ = 0;


// =============================================================================
// Public functions:

// Detects the number of cells for the attached battery up to 4 cells (it is
// impossible to detect the number of cells for batteries with 5 or more cells).
void DetectBattery(void)
{
  if (ADCState() == ADC_INACTIVE) return;

  // TODO: if (motors_on) return

  WaitOneADCCycle();
  ProcessSensorReadings();

  uint8_t n_cells;
  const uint8_t kMinVoltsPerCell = 33;  // 3.3 V is the LiPo operational minimum
  const uint8_t kMaxVoltsPerCell = 43;  // 4.3 V is max for LiPo batteries
  for (n_cells = 2; (n_cells < 5) && (BatteryVoltage() > (n_cells
    * kMaxVoltsPerCell - 2U)); n_cells++) continue;  // 0.2 V margin

  if (n_cells < 5) voltage_low_limit_ = n_cells * kMinVoltsPerCell;
  BeepNTimes(n_cells, 200);
  WaitForBuzzerToComplete();
}

// -----------------------------------------------------------------------------
uint8_t BatteryLow(void)
{
  return BatteryVoltage() < voltage_low_limit_;
}
