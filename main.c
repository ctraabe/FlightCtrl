#include <avr/interrupt.h>

#include "adc.h"
#include "attitude.h"
#include "battery.h"
#include "buzzer.h"
#include "control.h"
#include "i2c.h"
#include "led.h"
#include "mcu_pins.h"
#include "mk_serial_protocol.h"
#include "mk_serial_tx.h"
#include "motors.h"
#include "nav_comms.h"
#include "pressure_altitude.h"
#include "sbus.h"
#include "spi.h"
#include "state.h"
#include "timing.h"
#include "uart.h"
#include "vertical_speed.h"

#ifdef MOTOR_TEST
  #include "motor_test.h"
#endif
// TODO: remove
#include "union_types.h"
#include <util/crc16.h>


// ============================================================================+
// Private data:

static volatile uint8_t flag_128hz_ = 0, flag_64hz_ = 0, flag_2hz_ = 0;
static volatile uint16_t main_overrun_count_ = 0;


// =============================================================================
// Private function declarations:

int16_t main(void) __attribute__ ((noreturn));
void ResetOverrun(void);


// =============================================================================
// Public functions:

void PreflightInit(void)
{
  if (!MotorsInhibited()) return;
  BeepDuration(100);
  ZeroGyros();
  ResetPressureSensorRange();
  ResetAttitude();
  BeepDuration(500);
  WaitForBuzzerToComplete();
  ResetOverrun();
}

// -----------------------------------------------------------------------------
void SensorCalibration(void)
{
  if (!MotorsInhibited()) return;
  BeepDuration(100);
  ZeroAccelerometers();
  // PressureSensorBiasCalibration();  // Not accurate
  ResetAttitude();
  BeepDuration(500);
  WaitForBuzzerToComplete();
  ResetOverrun();
}


// =============================================================================
// Private functions:

static void Init(void)
{
  RedLEDOn();
  TimingInit();
  LEDInit();
  BuzzerInit();
  I2CInit();
  UARTInit();
  SPIInit();
  SBusInit();
  PressureSensorInit();

  // Pull up the version pin (FlightCtrl V2.2 will be grounded).
  VERSION_2_2_PORT |= VERSION_2_2_PIN;

  sei();  // Enable interrupts

  UARTPrintf("\n\rUniversity of Tokyo FlightCtrl firmware V2\n\r");

  LoadGyroOffsets();
  LoadAccelerometerOffsets();
  ADCOn();  // Start reading the sensors

  ResetPressureSensorRange();

  DetectBattery();
  DetectMotors();
  ControlInit();  // Must be run after DetectMotors() to get NMotors()

  NavCommsInit();

  ResetOverrun();
  GreenLEDOn();
}

// -----------------------------------------------------------------------------
void ErrorCheck(void)
{
  // Order from lowest to highest priority.
  if (main_overrun_count_ > 10) BeepPattern(0x000000AA);

  if (BatteryLow()) BeepPattern(0x000000CC);

  static uint8_t sbus_stale_pv = 0;
  if (SBusStale() && (!MotorsInhibited() || !sbus_stale_pv))
    BeepPattern(0x0CFCFCFC);  // dit dah dah dah
  if (sbus_stale_pv && !SBusStale())
    BeepDuration(500);

  sbus_stale_pv = SBusStale();
}

// -----------------------------------------------------------------------------
void ResetOverrun(void)
{
  flag_128hz_ = 0;
  main_overrun_count_ = 0;
  RedLEDOff();
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Init();

#ifdef MOTOR_TEST
  MotorTest();
  ResetOverrun();
#endif
/*
  // TODO: Delete these temporary EEPROM settings.
  SBusSetChannels(2, 3, 0, 1, 17, 8, 9, 10, 11, 16, 13, 4, 5, 6, 7);
  SetNMotors(4);
  float b_inv[8][4] = {
    { 1.899006944e+00, 1.899006944e+00, -3.818934998e+01, -3.966096111e+01 },
    { -1.899006944e+00, -1.899006944e+00, -3.818934998e+01, -3.966096111e+01 },
    { -1.899006944e+00, 1.899006944e+00, 3.818934998e+01, -3.966096111e+01 },
    { 1.899006944e+00, -1.899006944e+00, 3.818934998e+01, -3.966096111e+01 },
  };
  SetActuationInverse(b_inv);

  SetNMotors(8);
  float b_inv[8][4] = {
    { -7.032335119e-16, 9.502595253e+00, 2.219407750e+02, -5.337544437e+01 },
    { 8.318877305e+00, 6.719349542e+00, -2.219407750e+02, -5.337544437e+01 },
    { 1.176466911e+01, -2.211172238e-16, 2.219407750e+02, -5.337544437e+01 },
    { 8.318877305e+00, -6.719349542e+00, -2.219407750e+02, -5.337544437e+01 },
    { -7.032335119e-16, -9.502595253e+00, 2.219407750e+02, -5.337544437e+01 },
    { -8.318877305e+00, -6.719349542e+00, -2.219407750e+02, -5.337544437e+01 },
    { -1.176466911e+01, -2.211172238e-16, 2.219407750e+02, -5.337544437e+01 },
    { -8.318877305e+00, 6.719349542e+00, -2.219407750e+02, -5.337544437e+01 },
  };
  SetActuationInverse(b_inv);
*/
  // Main loop
  for (;;)  // Preferred over while(1)
  {
    if (flag_128hz_)
    {
      // NotifyNav();

      UpdateSBus();
      UpdateState();

      ProcessSensorReadings();

      UpdateAttitude();
      UpdatePressureAltitude();
      UpdateVerticalSpeed();

      Control();

      ErrorCheck();

      ProcessIncomingUART();
      SendPendingUART();

      if (main_overrun_count_) RedLEDOn();

      flag_128hz_ = 0;
    }

    if (NavRecieved()) ProcessDataFromNav();

    if (flag_64hz_)
    {
      flag_64hz_ = 0;

      static float theta = 0.0;
      theta += 2.0 * M_PI / 64.0;
      if (theta > M_PI) theta -= 2.0 * M_PI;
      float accel = sin(theta);
      ExchangeDataWithNav(accel / 9.8);

      static float velocity = 0.0;
      velocity += accel / 64.0;

      struct FromVision {
        uint16_t latency;  // Latency (ms)
        uint32_t capture_time;
        uint16_t reliability;  // TBD
        float velocity[3];  // (mm/frame)
        float quaternion[3];  // [q_x, q_y, q_z]
        float angular_velocity[3];  // (rad/frame)
        float position[3];  // (mm)
        uint16_t latency_ranging;  // (ms)
        float nearest_point_parameters[3];  // Distance and two angles, TBD
        float marking_point_parameters[3];  // Distance and two angles, TBD
      } __attribute__((packed)) * from_vision_ptr;

      uint8_t * tx_buffer = RequestUARTTxBuffer();
      if (tx_buffer)
      {
        uint8_t * temp_ptr;

        tx_buffer[0] = 0xFE;
        tx_buffer[1] = sizeof(struct FromVision);
        tx_buffer[2] = 'a';

        from_vision_ptr = (struct FromVision *)&tx_buffer[3];
        temp_ptr = (uint8_t * )from_vision_ptr;
        for (uint8_t i = sizeof(struct FromVision); i; --i) *temp_ptr++ = 0;
        from_vision_ptr->reliability = 1;
        from_vision_ptr->velocity[0] = velocity * 1000.0 / 30.0;

        union U16Bytes * crc = (union U16Bytes *)&tx_buffer[3 + sizeof(struct FromVision)];
        crc->u16 = 0xFFFF;
        temp_ptr = &tx_buffer[1];  // Skip the start byte
        for (uint8_t i = sizeof(struct FromVision) + 2; i; --i)
          crc->u16 = _crc_ccitt_update(crc->u16, *temp_ptr++);
        UARTTxBuffer(sizeof(struct FromVision) + 5);
      }
    }

    if (flag_2hz_)
    {
      flag_2hz_ = 0;
    }
  }
}

// -----------------------------------------------------------------------------
// This function is called upon the interrupt that occurs when TIMER3 reaches
// the value in ICR3. This should occur at a rate of 128 Hz.
ISR(TIMER3_CAPT_vect)
{
  enum {
    COUNTER_128HZ = 0xFF >> 7,
    COUNTER_64HZ = 0xFF >> 6,
    COUNTER_32HZ = 0xFF >> 5,
    COUNTER_16HZ = 0xFF >> 4,
    COUNTER_8HZ = 0xFF >> 3,
    COUNTER_4HZ = 0xFF >> 2,
    COUNTER_2HZ = 0xFF >> 1,
    COUNTER_1HZ = 0xFF >> 0,
  };

  sei();  // Allow other interrupts to be serviced

  static uint8_t counter = 0;
  switch ((uint8_t)(counter ^ (counter + 1))) {
    case COUNTER_1HZ:
    case COUNTER_2HZ:
      flag_2hz_ = 1;
    case COUNTER_4HZ:
    case COUNTER_8HZ:
    case COUNTER_16HZ:
      UpdateBuzzer();
    case COUNTER_32HZ:
    case COUNTER_64HZ:
      flag_64hz_ = 1;
    case COUNTER_128HZ:
      if (flag_128hz_)
        main_overrun_count_++;
      else
        flag_128hz_ = 1;
    default:
      counter++;
      break;
  }
}
