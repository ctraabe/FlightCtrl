#include "nav_comms.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/crc16.h>

#include "adc.h"
#include "attitude.h"
#include "control.h"
#include "main.h"
#include "state.h"
#include "sbus.h"
#include "spi.h"
#include "timing.h"
#include "union_types.h"


// =============================================================================
// Private data:

#define NAV_MESSAGE_START_BYTE (0xAA)

enum NavCommsState {
  NAV_COMMS_IDLE = 0,
  NAV_COMMS_DATA_READY,
  NAV_COMMS_DATA_IN_PROGRESS,
  NAV_COMMS_DATA_RECEVIED,
};

static volatile struct FromNav {
    uint16_t version;
    float position[3];
    float velocity[3];
    float heading_correction_quat_0;
    float heading_correction_quat_z;
    uint16_t status;
    uint16_t crc;
} __attribute__((packed)) from_nav_[2];

static volatile uint8_t nav_data_state_ = NAV_COMMS_IDLE;
static uint8_t from_nav_head_ = 1, from_nav_tail_ = 0;


// =============================================================================
// Private function declarations:

static void SetNavDataReceived(void);


// =============================================================================
// Accessors:

uint8_t NavDataReady(void)
{
  return nav_data_state_ == NAV_COMMS_DATA_READY;
}

// -----------------------------------------------------------------------------
uint8_t NavRecieved(void)
{
  return nav_data_state_ == NAV_COMMS_DATA_RECEVIED;
}

// -----------------------------------------------------------------------------
const volatile float * Position(void)
{
  return from_nav_[from_nav_tail_].position;
}

// -----------------------------------------------------------------------------
const volatile float * Velocity(void)
{
  return from_nav_[from_nav_tail_].velocity;
}

// -----------------------------------------------------------------------------
float HeadingCorrection(void)
{
  return from_nav_[from_nav_tail_].heading_correction_quat_z;
}


// =============================================================================
// Public functions:

void NavCommsInit(void)
{
  // Pull up pin PC4.
  PORTC |= _BV(4);

  // Enable pin change interrupts.
  PCICR |= _BV(PCIE2);
}

// -----------------------------------------------------------------------------
void ExchangeDataWithNav(void)
{
  // Request the SPI transmit buffer. Return if not available.
  uint8_t * tx_buffer = RequestSPITxBuffer();
  if (tx_buffer == 0) return;

  // Specify the payload structure.
  struct ToNav {
    uint16_t timestamp;
    uint16_t state;
    float accelerometer[3];
    float gyro[3];
    float quaternion[4];
#ifdef LOG_FLT_CTRL_DEBUG_TO_SD
    int16_t sbus_pitch;
    int16_t sbus_roll;
    int16_t sbus_yaw;
    int16_t sbus_thrust;
    uint16_t biased_pressure;
    uint16_t battery_voltage;
    float heading_command;
    float attitude_integral[3];
    float quaternion_model[4];
    float angular_command[3];
    uint16_t motor_setpoints[8];
#endif
  } __attribute__((packed));

  _Static_assert(2 + sizeof(struct ToNav) + 2 + 4 < SPI_TX_BUFFER_LENGTH,
    "struct ToNav is too large for the SPI TX buffer");

  // Populate the transmit buffer.
  tx_buffer[0] = NAV_MESSAGE_START_BYTE;
  tx_buffer[1] = sizeof(struct ToNav);

  struct ToNav * to_nav_ptr = (struct ToNav *)&tx_buffer[2];

  to_nav_ptr->timestamp = GetTimestamp();
  to_nav_ptr->state = State();
  to_nav_ptr->accelerometer[0] = AccelerationVector()[X_BODY_AXIS];
  to_nav_ptr->accelerometer[1] = AccelerationVector()[Y_BODY_AXIS];
  to_nav_ptr->accelerometer[2] = AccelerationVector()[Z_BODY_AXIS];
  to_nav_ptr->gyro[0] = AngularRateVector()[X_BODY_AXIS];
  to_nav_ptr->gyro[1] = AngularRateVector()[Y_BODY_AXIS];
  to_nav_ptr->gyro[2] = AngularRateVector()[Z_BODY_AXIS];
  to_nav_ptr->quaternion[0] = Quat()[0];
  to_nav_ptr->quaternion[1] = Quat()[1];
  to_nav_ptr->quaternion[2] = Quat()[2];
  to_nav_ptr->quaternion[3] = Quat()[3];
#ifdef LOG_FLT_CTRL_DEBUG_TO_SD
  to_nav_ptr->sbus_pitch = SBusPitch();
  to_nav_ptr->sbus_roll = SBusRoll();
  to_nav_ptr->sbus_yaw = SBusYaw();
  to_nav_ptr->sbus_thrust = SBusThrust();
  to_nav_ptr->battery_voltage = BatteryVoltage();
  to_nav_ptr->battery_voltage = BiasedPressureSum();
  to_nav_ptr->heading_command = HeadingCommand();
  to_nav_ptr->attitude_integral[0] = AttitudeIntegralVector()[0];
  to_nav_ptr->attitude_integral[1] = AttitudeIntegralVector()[1];
  to_nav_ptr->attitude_integral[2] = AttitudeIntegralVector()[2];
  to_nav_ptr->quaternion_model[0] = QuatModelVector()[0];
  to_nav_ptr->quaternion_model[1] = QuatModelVector()[1];
  to_nav_ptr->quaternion_model[2] = QuatModelVector()[2];
  to_nav_ptr->quaternion_model[3] = QuatModelVector()[3];
  to_nav_ptr->angular_command[0] = AngularCommand(0);
  to_nav_ptr->angular_command[1] = AngularCommand(1);
  to_nav_ptr->angular_command[2] = AngularCommand(2);
  to_nav_ptr->motor_setpoints[0] = MotorSetpoint(0);
  to_nav_ptr->motor_setpoints[1] = MotorSetpoint(1);
  to_nav_ptr->motor_setpoints[2] = MotorSetpoint(2);
  to_nav_ptr->motor_setpoints[3] = MotorSetpoint(3);
  to_nav_ptr->motor_setpoints[4] = MotorSetpoint(4);
  to_nav_ptr->motor_setpoints[5] = MotorSetpoint(5);
  to_nav_ptr->motor_setpoints[6] = MotorSetpoint(6);
  to_nav_ptr->motor_setpoints[7] = MotorSetpoint(7);
#endif

  // Add a CRC just after the data payload. Note that this is the same CRC that
  // is used in MAVLink. It uses a polynomial represented by 0x1021, an initial
  // value of 0xFFFF, and with both the input and output reflected.
  union U16Bytes * crc = (union U16Bytes *)&tx_buffer[2 + sizeof(struct ToNav)];
  crc->u16 = 0xFFFF;
  uint8_t * tx_buffer_ptr = &tx_buffer[1];  // Skip the start byte
  for (uint8_t i = sizeof(struct ToNav) + 1; i--; )
    crc->u16 = _crc_ccitt_update(crc->u16, *tx_buffer_ptr++);

  // The MikroKopter NaviCtrl board employs a STR91x microcontroller. The SPI Rx
  // interrupt for this board only occurs when the Rx buffer (8 bytes) is more
  // than half full. To make sure that this interrupt is triggered, add 4
  // trailing bytes to the end of the transmission.
  tx_buffer_ptr += sizeof(uint16_t);  // Skip the CRC
  for (uint8_t i = 4; i--; ) *tx_buffer_ptr++ = 0x00;

  // Send the entire packet (1-byte header, 1-byte length, payload, 2-byte CRC &
  // 4-bytes trailing zeros).
  SPIExchangeThenCallback(2 + sizeof(struct ToNav) + 2 + 4,
    (volatile uint8_t *)&from_nav_[from_nav_head_], sizeof(struct FromNav),
    SetNavDataReceived);

  nav_data_state_ = NAV_COMMS_DATA_IN_PROGRESS;
}

// -----------------------------------------------------------------------------
void NotifyNav(void)
{
  // Disable the pin change interrupt for PC4.
  PCMSK2 &= ~_BV(PCINT20);

  // Pull down PC4.
  PORTC &= ~_BV(4);
  DDRC |= _BV(4);

  asm ("nop");  // Wait one cycle

  // Set PC4 back to input and pull-up.
  DDRC &= ~_BV(4);
  PORTC |= _BV(4);

  // Re-enable the pin change interrupt for pin PC4.
  PCMSK2 |= _BV(PCINT20);
}

// -----------------------------------------------------------------------------
void ProcessDataFromNav(void)
{
  volatile uint8_t * rx_buffer_ptr =
    (volatile uint8_t *)&from_nav_[from_nav_head_];

  uint16_t crc = 0xFFFF;
  for (uint8_t i = sizeof(struct FromNav) - 2; i--; )
    crc = _crc_ccitt_update(crc, *(rx_buffer_ptr++));

  if (from_nav_[from_nav_head_].crc == crc)
  {
    // Swap buffers.
    from_nav_tail_ = from_nav_head_;
    from_nav_head_ = !from_nav_tail_;
  }

  nav_data_state_ = NAV_COMMS_IDLE;
}

// =============================================================================
// Private function:

static void SetNavDataReceived(void)
{
  nav_data_state_ = NAV_COMMS_DATA_RECEVIED;
}

// -----------------------------------------------------------------------------
ISR(PCINT2_vect)
{
  // Ignore rising edge.
  if (PINC & _BV(4)) return;

  nav_data_state_ = NAV_COMMS_DATA_READY;

  // Disable the interrupt.
  PCMSK2 &= ~_BV(PCINT20);
}