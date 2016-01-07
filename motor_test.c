// This file implements functionality intended to measure the response (i.e.
// rotational speed and transient performance) to motor commands. This
// functionality is enabled when the program is compiled with MOTOR_TEST
// defined.

// This test requires a motor revolution counter. The author of this file used a
// revolution counter that employed a small neodymium magnet and a hall effect
// sensor. The hall effect sensor normally output 5V, but the signal
// transitioned to ground when in the vicinity of the magnet. The magnet was
// placed on the rotating housing of the brushless motor so that the hall effect
// sensor would register a trough once per revolution. This file outputs the
// timestamp of each trough to the UART in plain ASCII. Also output to UART is
// the timestamp and value of each new command. With this data, the motor
// response to commands can be determined.

// Please note that this test should be performed with the actual propeller and
// battery combination to be used in flight as the response varies with these.

// The test is armed by starting up with the transmitter on and switch 0 in the
// up position. The armed state is indicated with a fast-blinking red LED. The
// test begins by moving switch 1 to the up position.

#include "motor_test.h"

#include <avr/interrupt.h>

#include "adc.h"
#include "led.h"
#include "mcu_pins.h"
#include "motors.h"
#include "sbus.h"
#include "timing.h"
#include "uart.h"


// ============================================================================+
// Private data:

static volatile uint16_t new_pulse = 0;
static volatile uint16_t pulse_timestamp = 0;


// =============================================================================
// Private function declarations:

static void MotorTestStep(uint16_t command, uint16_t duration_ms);


// =============================================================================
// Public functions:

void MotorTest(void)
{
  Wait(1000);
  UpdateSBus();
  if (SBusSwitch(0) < SBUS_MAX / 2) return;

  uint16_t switch1_pv = SBusSwitch(1);
  uint16_t delay = GetTimestampMillisFromNow(50);
  for (;;)
  {
    while (!TimestampInPast(delay)) continue;
    delay += 50;
    RedLEDToggle();
    UpdateSBus();
    if ((SBusSwitch(1) > SBUS_MAX / 2) && (switch1_pv < SBUS_MAX / 2)) break;
    switch1_pv = SBusSwitch(1);
  }
  RedLEDOff();

  // Turn off ADC (frequent interrupts).
  ADCOff();

  // Set external LED pins to input.
  EXTERNAL_LED_DDR &= ~EXTERNAL_LED_1_PIN & ~EXTERNAL_LED_3_PIN;

  // Enable the interrupt (INT2) on pin PB2 (J8) and set to either edge.
  EICRA = _BV(ISC20);
  EIMSK = _BV(INTF2);

  // Disable TIMER3 interrupts and reconfigure as 78125 Hz counter.
  TIMSK3 = 0x00;
  TCCR3B = (1 << CS32) | (0 << CS31) | (0 << CS30);

  TCNT3 = 0x0000;

  MotorTestStep(230,1000);
  MotorTestStep(1840,1000);
  MotorTestStep(230,1000);
  MotorTestStep(460,1000);
  MotorTestStep(690,1000);
  MotorTestStep(920,1000);
  MotorTestStep(1150,1000);
  MotorTestStep(1380,1000);
  MotorTestStep(1610,1000);
  MotorTestStep(1840,1000);
  MotorTestStep(1610,1000);
  MotorTestStep(1380,1000);
  MotorTestStep(1150,1000);
  MotorTestStep(920,1000);
  MotorTestStep(690,1000);
  MotorTestStep(460,1000);
  MotorTestStep(230,1000);
  MotorTestStep(0,100);

  // Revert the system state.
  EICRA = 0x00;
  EIMSK = 0x00;
  EXTERNAL_LED_DDR |= EXTERNAL_LED_1_PIN | EXTERNAL_LED_3_PIN;
  TimingInit();
  ADCOn();
}


// =============================================================================
// Private functions:

static void MotorTestStep(uint16_t command, uint16_t duration_ms)
{
  if (SBusSwitch(1) < SBUS_MAX / 2) return;

  SetMotorSetpoint(7, command);

  uint16_t start_time = GetTimestamp();
  TxMotorSetpoints();

  uint16_t timeout = start_time + duration_ms;
  uint16_t update_timer = start_time + 20;
  UARTPrintf("c%u,%u", TCNT3, command);

  for (;;)
  {
    if (new_pulse)
    {
      new_pulse = 0;
      UARTPrintf("x%u", pulse_timestamp);
    }

    if (TimestampInPast(update_timer))
    {
      TxMotorSetpoints();
      UpdateSBus();
      if (SBusSwitch(1) < SBUS_MAX / 2) return;
      update_timer += 20;
    }

    if (TimestampInPast(timeout)) break;
  }
}

// -----------------------------------------------------------------------------
ISR(INT2_vect)
{
  if (~PINB & _BV(2))
  {
    pulse_timestamp = TCNT3;
    new_pulse = 1;
    RedLEDOn();
  }
  else
  {
    RedLEDOff();
  }
}
