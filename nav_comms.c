#include "nav_comms.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/crc16.h>

#include "adc.h"
#include "attitude.h"
#include "control.h"
#include "main.h"
#include "pressure_altitude.h"
#include "state.h"
#include "sbus.h"
#include "spi.h"
#include "timing.h"
#include "union_types.h"
#include "vertical_speed.h"


// =============================================================================
// Private data:

#define NAV_MESSAGE_START_BYTE (0xAA)
#define NAV_FRESHNESS_LIMIT (500)  // millisends

static volatile struct FromNav {
    uint16_t version;
    float position[3];
    float velocity[3];
    float heading_correction_quat_0;
    float heading_correction_quat_z;
    float target_position[3];
    float transit_speed;
    float target_heading;
    float heading_rate;
    uint8_t nav_mode;
    uint8_t status;
    uint16_t crc;
} __attribute__((packed)) from_nav_[2] = { 0 };

static volatile enum NavCommsState {
  NAV_COMMS_IDLE = 0,
  NAV_COMMS_DATA_READY,
  NAV_COMMS_DATA_IN_PROGRESS,
  NAV_COMMS_DATA_RECEVIED,
} nav_data_state_ = NAV_COMMS_IDLE;

static enum NavModeBits {
  NAV_BIT_MODE_0     = 1<<0,
  NAV_BIT_MODE_1     = 1<<1,
  NAV_BIT_HOLD_RESET = 1<<2,
  NAV_BIT_RESERVED_0 = 1<<3,
  NAV_BIT_ROUTE_0    = 1<<4,
  NAV_BIT_ROUTE_1    = 1<<5,
  NAV_BIT_SWITCH_0   = 1<<6,
  NAV_BIT_SWITCH_1   = 1<<7,
} nav_mode_request_;

static uint8_t from_nav_head_ = 1, from_nav_tail_ = 0;
static uint16_t last_reception_timestamp_ = 0;
static enum NavErrorBits nav_error_bits_ = NAV_ERROR_BIT_STALE;


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
enum NavMode NavMode(void)
{
  return (enum NavMode)(from_nav_[from_nav_tail_].nav_mode & 0x03);
}

// -----------------------------------------------------------------------------
uint16_t NavStatus(void)
{
  return from_nav_[from_nav_tail_].status;
}

// -----------------------------------------------------------------------------
float HeadingCorrection0(void)
{
  return from_nav_[from_nav_tail_].heading_correction_quat_0;
}

// -----------------------------------------------------------------------------
float HeadingCorrectionZ(void)
{
  return from_nav_[from_nav_tail_].heading_correction_quat_z;
}

// -----------------------------------------------------------------------------
const volatile float * PositionVector(void)
{
  return from_nav_[from_nav_tail_].position;
}

// -----------------------------------------------------------------------------
const volatile float * VelocityVector(void)
{
  return from_nav_[from_nav_tail_].velocity;
}

// -----------------------------------------------------------------------------
const volatile float * TargetPositionVector(void)
{
  return from_nav_[from_nav_tail_].target_position;
}

// -----------------------------------------------------------------------------
float TransitSpeed(void)
{
  return from_nav_[from_nav_tail_].transit_speed;
}

// -----------------------------------------------------------------------------
float TargetHeading(void)
{
  return from_nav_[from_nav_tail_].target_heading;
}

// -----------------------------------------------------------------------------
float HeadingRate(void)
{
  return from_nav_[from_nav_tail_].heading_rate;
}

// -----------------------------------------------------------------------------
uint8_t NavStale(void)
{
  return nav_error_bits_ & NAV_ERROR_BIT_STALE;
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
  // Only check freshness if the data is not yet stale because the timestamp
  // might rollover, giving a false freshness.
  if ((~nav_error_bits_ & NAV_ERROR_BIT_STALE) &&
    (MillisSinceTimestamp(last_reception_timestamp_) > NAV_FRESHNESS_LIMIT))
  {
    nav_error_bits_ |= NAV_ERROR_BIT_STALE;
  }

  // Request the SPI transmit buffer. Return if not available.
  uint8_t * tx_buffer = RequestSPITxBuffer();
  if (tx_buffer == 0) return;

  // Specify the payload structure.
  struct ToNav {
    uint16_t timestamp;
    uint8_t nav_mode_request;
    uint8_t state;
    float accelerometer[3];
    float gyro[3];
    float quaternion[4];
    float pressure_altitude;
#ifdef LOG_FLT_CTRL_DEBUG_TO_SD
    int16_t sbus_pitch;
    int16_t sbus_roll;
    int16_t sbus_yaw;
    int16_t sbus_thrust;
    uint16_t battery_voltage;
    float thrust_command;
    float heading_command;
    float angular_command[3];
    float kalman_p_dot;
    float kalman_q_dot;
    float vertical_speed;
    float vertical_acceleration;
#endif
  } __attribute__((packed));

  _Static_assert(2 + sizeof(struct ToNav) + 2 + 4 < SPI_TX_BUFFER_LENGTH,
    "struct ToNav is too large for the SPI TX buffer");

  // Populate the transmit buffer.
  tx_buffer[0] = NAV_MESSAGE_START_BYTE;
  tx_buffer[1] = sizeof(struct ToNav);

  struct ToNav * to_nav_ptr = (struct ToNav *)&tx_buffer[2];

  to_nav_ptr->timestamp = GetTimestamp();
  to_nav_ptr->nav_mode_request = nav_mode_request_ | NavModeRequest()
    | (SBusSwitch(0) << 4) | (SBusSwitch(1) << 6);
  to_nav_ptr->state = State();
  to_nav_ptr->accelerometer[0] = Acceleration(X_BODY_AXIS);
  to_nav_ptr->accelerometer[1] = Acceleration(Y_BODY_AXIS);
  to_nav_ptr->accelerometer[2] = Acceleration(Z_BODY_AXIS);
  to_nav_ptr->gyro[0] = AngularRate(X_BODY_AXIS);
  to_nav_ptr->gyro[1] = AngularRate(Y_BODY_AXIS);
  to_nav_ptr->gyro[2] = AngularRate(Z_BODY_AXIS);
  to_nav_ptr->quaternion[0] = Quat()[0];
  to_nav_ptr->quaternion[1] = Quat()[1];
  to_nav_ptr->quaternion[2] = Quat()[2];
  to_nav_ptr->quaternion[3] = Quat()[3];
  to_nav_ptr->pressure_altitude = DeltaPressureAltitude();
#ifdef LOG_FLT_CTRL_DEBUG_TO_SD
  to_nav_ptr->sbus_pitch = SBusPitch();
  to_nav_ptr->sbus_roll = SBusRoll();
  to_nav_ptr->sbus_yaw = SBusYaw();
  to_nav_ptr->sbus_thrust = SBusThrust();
  to_nav_ptr->battery_voltage = BatteryVoltage();
  to_nav_ptr->thrust_command = ThrustCommand();
  to_nav_ptr->heading_command = HeadingCommand();
  to_nav_ptr->angular_command[0] = AngularCommand(0);
  to_nav_ptr->angular_command[1] = AngularCommand(1);
  to_nav_ptr->angular_command[2] = AngularCommand(2);
  to_nav_ptr->kalman_p_dot = KalmanPDot();
  to_nav_ptr->kalman_q_dot = KalmanQDot();
  to_nav_ptr->vertical_speed = VerticalSpeed();
  to_nav_ptr->vertical_acceleration = VerticalAcceleration();
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
    // Clear the stale data bit.
    nav_error_bits_ &= ~NAV_ERROR_BIT_STALE;
    last_reception_timestamp_ = GetTimestamp();
  }

  // Clear the nav hold reset request if it has been honored.
  // TODO: maybe this should get handled elsewhere?
  if (from_nav_[from_nav_tail_].nav_mode & NAV_BIT_HOLD_RESET)
    nav_mode_request_ &= ~NAV_BIT_HOLD_RESET;

  nav_data_state_ = NAV_COMMS_IDLE;
}

// -----------------------------------------------------------------------------
void RequestNavRoute(uint8_t nav_route)
{
  if (nav_route > 0x03) nav_route = 0x03;
  nav_mode_request_ |= nav_route << 4;
}

// -----------------------------------------------------------------------------
void ResetPositionHold(void)
{
  nav_mode_request_ |= NAV_BIT_HOLD_RESET;
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