#include "sbus.h"

#include <stdlib.h>
#include <avr/interrupt.h>

#include "eeprom.h"
#include "timing.h"


// =============================================================================
// Private data:

#define USART1_BAUD (100000L)
#define SBUS_NO_NEW_DATA (-1)
#define SBUS_SIGNAL_LOST_BIT (2)
#define SBUS_STICK_EDGE_THRESHOLD (SBUS_MAX / 25)  // 2%
#define SBUS_STICK_CENTER_THRESHOLD (SBUS_MAX / 10)  // 10% on either side
#define SBUS_FRESHNESS_LIMIT (100)  // millisends

// The following is not declared static so that it will be visible to sbus.S.
volatile uint8_t sbus_rx_buffer_[2][SBUS_RX_BUFFER_LENGTH];
volatile int8_t sbus_data_ready_ = SBUS_NO_NEW_DATA;

static struct SBusData
{
  uint16_t timestamp;
  int16_t channels[12];
  uint8_t binary;
} __attribute__((packed)) sbus_data_;

static uint8_t sbus_error_bits_ = 0x00;
static uint8_t channel_pitch_, channel_roll_, channel_yaw_, channel_thrust_,
  channel_on_off_, channel_altitude_control_, channel_nav_control_,
  channel_takeoff_, channel_switch_[6], channel_trim_[4];


// =============================================================================
// Private function declarations:

static inline uint8_t SBusByte(uint8_t);
static enum SBusSwitchState SBusChannelSwitchState(uint8_t channel);


// =============================================================================
// Accessors:

uint8_t SBusErrorBits(void)
{
  return sbus_error_bits_;
}

// -----------------------------------------------------------------------------
int16_t SBusPitch(void)
{
  return sbus_data_.channels[channel_pitch_];
}

// -----------------------------------------------------------------------------
int16_t SBusRoll(void)
{
  return sbus_data_.channels[channel_roll_];
}

// -----------------------------------------------------------------------------
int16_t SBusYaw(void)
{
  return sbus_data_.channels[channel_yaw_];
}

// -----------------------------------------------------------------------------
int16_t SBusThrust(void)
{
  return sbus_data_.channels[channel_thrust_];
}

// -----------------------------------------------------------------------------
uint8_t SBusOnOff(void)
{
  return SBusChannelSwitchState(channel_on_off_) == SBUS_SWITCH_UP;
}

// -----------------------------------------------------------------------------
enum SBusSwitchState SBusAltitudeControl(void)
{
  return SBusChannelSwitchState(channel_altitude_control_);
}

// -----------------------------------------------------------------------------
enum SBusSwitchState SBusNavControl(void)
{
  return SBusChannelSwitchState(channel_nav_control_);
}

// -----------------------------------------------------------------------------
enum SBusSwitchState SBusTakeoff(void)
{
  return SBusChannelSwitchState(channel_takeoff_);
}

// -----------------------------------------------------------------------------
enum SBusSwitchState SBusSwitch(uint8_t i)
{
  return SBusChannelSwitchState(channel_switch_[i]);
}

// -----------------------------------------------------------------------------
int16_t SBusTrim(uint8_t i)
{
  return sbus_data_.channels[channel_trim_[i]];
}

// -----------------------------------------------------------------------------
uint8_t SBusStale(void)
{
  return sbus_error_bits_ & SBUS_ERROR_BIT_STALE;
}


// =============================================================================
// Public functions:

void SBusInit(void)
{
  // Set the baud rate.
  UBRR1 = F_CPU / 8L / USART1_BAUD - 1;
  // Set UART Double Speed (U2X).
  UCSR1A = (1 << U2X1);
  // Enable USART1 receiver and transmitter and interrupts.
  UCSR1B = (1 << RXCIE1)  // RX Complete Interrupt Enable
         | (0 << TXCIE1)  // TX Complete Interrupt Enable
         | (0 << UDRIE1)  // Data Register Empty Interrupt Enable
         | (0 << TXEN1)  // Transmitter Enable
         | (1 << RXEN1)  // Receiver Enable
         | (0 << UCSZ12);  // 9-bit Character Size Enable
  UCSR1C = (0 << UMSEL11) | (0 << UMSEL10)  // USART Mode (asynchronous)
         | (1 << UPM11) | (0 << UPM10)  // Parity Bit Mode (even)
         | (1 << USBS1)  // 2 Stop Bit Enable
         | (1 << UCSZ11) | (1 << UCSZ10);  // Character Size (8-bits)

  channel_pitch_ = eeprom_read_byte(&eeprom.sbus_channel_pitch);
  channel_roll_ = eeprom_read_byte(&eeprom.sbus_channel_roll);
  channel_yaw_ = eeprom_read_byte(&eeprom.sbus_channel_yaw);
  channel_thrust_ = eeprom_read_byte(&eeprom.sbus_channel_thrust);
  channel_on_off_ = eeprom_read_byte(&eeprom.sbus_channel_on_off);
  channel_altitude_control_ =
    eeprom_read_byte(&eeprom.sbus_channel_altitude_control);
  channel_nav_control_ = eeprom_read_byte(&eeprom.sbus_channel_nav_control);
  channel_takeoff_ = eeprom_read_byte(&eeprom.sbus_channel_takeoff);
  eeprom_read_block((void*)channel_switch_,
    (const void*)&eeprom.sbus_channel_switch[0], sizeof(channel_switch_));
  eeprom_read_block((void*)channel_trim_,
    (const void*)&eeprom.sbus_channel_trim[0], sizeof(channel_trim_));
}

// -----------------------------------------------------------------------------
void SBusSetChannels(uint8_t pitch, uint8_t roll, uint8_t yaw, uint8_t thrust,
  uint8_t on_off, uint8_t altitude_control, uint8_t nav_control,
  uint8_t takeoff, uint8_t switch0, uint8_t switch1, uint8_t switch2,
  uint8_t switch3, uint8_t switch4, uint8_t switch5, uint8_t trim0,
  uint8_t trim1, uint8_t trim2, uint8_t trim3)
{
  channel_pitch_ = pitch;
  channel_roll_ = roll;
  channel_yaw_ = yaw;
  channel_thrust_ = thrust;
  channel_on_off_ = on_off;
  channel_altitude_control_ = altitude_control;
  channel_nav_control_ = nav_control;
  channel_takeoff_ = takeoff;
  channel_switch_[0] = switch0;
  channel_switch_[1] = switch1;
  channel_switch_[2] = switch2;
  channel_switch_[3] = switch3;
  channel_switch_[4] = switch4;
  channel_switch_[5] = switch5;
  channel_trim_[0] = trim0;
  channel_trim_[1] = trim1;
  channel_trim_[2] = trim2;
  channel_trim_[3] = trim3;
  eeprom_update_byte(&eeprom.sbus_channel_pitch, pitch);
  eeprom_update_byte(&eeprom.sbus_channel_roll, roll);
  eeprom_update_byte(&eeprom.sbus_channel_yaw, yaw);
  eeprom_update_byte(&eeprom.sbus_channel_thrust, thrust);
  eeprom_update_byte(&eeprom.sbus_channel_on_off, on_off);
  eeprom_update_byte(&eeprom.sbus_channel_altitude_control, altitude_control);
  eeprom_update_byte(&eeprom.sbus_channel_nav_control, nav_control);
  eeprom_update_byte(&eeprom.sbus_channel_takeoff, takeoff);
  eeprom_update_block((const void*)channel_switch_,
    (void*)&eeprom.sbus_channel_switch[0], sizeof(channel_switch_));
  eeprom_update_block((const void*)channel_trim_,
    (void*)&eeprom.sbus_channel_trim[0], sizeof(channel_trim_));
}

// -----------------------------------------------------------------------------
uint8_t SBusPitchStickCentered(void)
{
  return SBusStale() || abs(SBusPitch()) < SBUS_STICK_CENTER_THRESHOLD;
}

// -----------------------------------------------------------------------------
uint8_t SBusRollStickCentered(void)
{
  return SBusStale() || abs(SBusRoll()) < SBUS_STICK_CENTER_THRESHOLD;
}

// -----------------------------------------------------------------------------
uint8_t SBusThrustStickCentered(void)
{
  return SBusStale() || abs(SBusThrust()) < SBUS_STICK_CENTER_THRESHOLD;
}

// -----------------------------------------------------------------------------
uint8_t SBusThrustStickDown(void)
{
  return !SBusStale() && SBusThrust() < -(SBUS_MAX - SBUS_STICK_EDGE_THRESHOLD);
}

// -----------------------------------------------------------------------------
uint8_t SBusThrustStickUp(void)
{
  return !SBusStale() && SBusThrust() > SBUS_MAX - SBUS_STICK_EDGE_THRESHOLD;
}

// -----------------------------------------------------------------------------
uint8_t SBusYawStickCentered(void)
{
  return SBusStale() || abs(SBusYaw()) < SBUS_STICK_CENTER_THRESHOLD;
}

// -----------------------------------------------------------------------------
uint8_t SBusYawStickLeft(void)
{
  return !SBusStale() && SBusYaw() > SBUS_MAX - SBUS_STICK_EDGE_THRESHOLD;
}

// -----------------------------------------------------------------------------
uint8_t SBusYawStickRight(void)
{
  return !SBusStale() && SBusYaw() < -(SBUS_MAX - SBUS_STICK_EDGE_THRESHOLD);
}

// -----------------------------------------------------------------------------
void UpdateSBus(void)
{
  // Only check freshness if the data is not yet stale because the timestamp
  // might rollover, giving a false freshness.
  if ((~sbus_error_bits_ & SBUS_ERROR_BIT_STALE) &&
    (MillisSinceTimestamp(sbus_data_.timestamp) > SBUS_FRESHNESS_LIMIT))
  {
    sbus_error_bits_ |= SBUS_ERROR_BIT_STALE;
  }

  if (sbus_data_ready_ == SBUS_NO_NEW_DATA) return;

  // Make a local (non-volatile) pointer to the appropriate side of
  // sbus_rx_buffer_.
  uint8_t* rx_buffer = (uint8_t*)&sbus_rx_buffer_[sbus_data_ready_][0];

  // Reset the sbus_data_ready_ indicator.
  sbus_data_ready_ = SBUS_NO_NEW_DATA;

  // Check that the received data is valid.
  if ((rx_buffer[SBusByte(24)] != SBUS_END_BYTE)
    || (rx_buffer[SBusByte(23)] & (1 << SBUS_SIGNAL_LOST_BIT))) return;

  // Record the timestamp
  ((uint8_t*)&sbus_data_.timestamp)[0] = rx_buffer[SBUS_RX_BUFFER_LENGTH - 2];
  ((uint8_t*)&sbus_data_.timestamp)[1] = rx_buffer[SBUS_RX_BUFFER_LENGTH - 1];

  sbus_data_.binary = rx_buffer[SBusByte(23)];

  // Channel 0 is [bytes(2)(2:0) bytes(1)(7:0)]
  ((uint8_t*)&sbus_data_.channels[0])[1] = rx_buffer[SBusByte(2)] & 0x07;
  ((uint8_t*)&sbus_data_.channels[0])[0] = rx_buffer[SBusByte(1)];

  // Channel 1 is [bytes(3)(5:0) bytes(2)(7:3)]
  ((uint8_t*)&sbus_data_.channels[1])[1] = rx_buffer[SBusByte(3)];
  ((uint8_t*)&sbus_data_.channels[1])[0] = rx_buffer[SBusByte(2)];
  sbus_data_.channels[1] >>= 3;

  // Channel 2 is [bytes(5)(0:0), bytes(4)(7:0), bytes(3)(7:6)]
  ((uint8_t*)&sbus_data_.channels[2])[1] = rx_buffer[SBusByte(5)];
  ((uint8_t*)&sbus_data_.channels[2])[0] = rx_buffer[SBusByte(4)];
  sbus_data_.channels[2] <<= 2;
  ((uint8_t*)&sbus_data_.channels[2])[0]
    |= (((uint8_t*)&sbus_data_.channels[1])[1] & 0x1F) >> 3;
  ((uint8_t*)&sbus_data_.channels[2])[1] &= 0x07;

  ((uint8_t*)&sbus_data_.channels[1])[1] &= 0x07;

  // // Channel 3 is [bytes(6)(3:0), bytes(5)(7:1)]
  ((uint8_t*)&sbus_data_.channels[3])[1] = rx_buffer[SBusByte(6)];
  ((uint8_t*)&sbus_data_.channels[3])[0] = rx_buffer[SBusByte(5)];
  sbus_data_.channels[3] >>= 1;
  ((uint8_t*)&sbus_data_.channels[3])[1] &= 0x07;

  // Channel 4 is [bytes(7)(6:0), bytes(6)(7:4)]
  ((uint8_t*)&sbus_data_.channels[4])[1] = rx_buffer[SBusByte(7)];
  ((uint8_t*)&sbus_data_.channels[4])[0] = rx_buffer[SBusByte(6)];
  sbus_data_.channels[4] >>= 4;

  // Channel 5 is [bytes(9)(1:0), bytes(8)(7:0), bytes(7)(7:7)]
  ((uint8_t*)&sbus_data_.channels[5])[1] = rx_buffer[SBusByte(9)];
  ((uint8_t*)&sbus_data_.channels[5])[0] = rx_buffer[SBusByte(8)];
  sbus_data_.channels[5] <<= 1;
  ((uint8_t*)&sbus_data_.channels[5])[0]
    |= (((uint8_t*)&sbus_data_.channels[4])[1] & 0x0F) >> 3;
  ((uint8_t*)&sbus_data_.channels[5])[1] &= 0x07;

  ((uint8_t*)&sbus_data_.channels[4])[1] &= 0x07;

  // Channel 6 is [bytes(10)(4:0), bytes(9)(7:2)]
  ((uint8_t*)&sbus_data_.channels[6])[1] = rx_buffer[SBusByte(10)];
  ((uint8_t*)&sbus_data_.channels[6])[0] = rx_buffer[SBusByte(9)];
  sbus_data_.channels[6] >>= 2;
  ((uint8_t*)&sbus_data_.channels[6])[1] &= 0x07;

  // Channel 7 is [bytes(11)(7:0), bytes(10)(7:5)]
  ((uint8_t*)&sbus_data_.channels[7])[1] = rx_buffer[SBusByte(11)];
  ((uint8_t*)&sbus_data_.channels[7])[0] = rx_buffer[SBusByte(10)];
  sbus_data_.channels[7] >>= 5;
  ((uint8_t*)&sbus_data_.channels[7])[1] &= 0x07;

  // Channel 8 is [bytes(13)(2:0), bytes(12)(7:0)]
  ((uint8_t*)&sbus_data_.channels[8])[1] = rx_buffer[SBusByte(13)] & 0x07;
  ((uint8_t*)&sbus_data_.channels[8])[0] = rx_buffer[SBusByte(12)];

  // Channel 9 is [bytes(14)(5:0), bytes(13)(7:3)]
  ((uint8_t*)&sbus_data_.channels[9])[1] = rx_buffer[SBusByte(14)];
  ((uint8_t*)&sbus_data_.channels[9])[0] = rx_buffer[SBusByte(13)];
  sbus_data_.channels[9] >>= 3;

  // Channel 10 is [bytes(16)(0:0), bytes(15)(7:0), bytes(14)(7:6)]
  ((uint8_t*)&sbus_data_.channels[10])[1] = rx_buffer[SBusByte(16)];
  ((uint8_t*)&sbus_data_.channels[10])[0] = rx_buffer[SBusByte(15)];
  sbus_data_.channels[10] <<= 2;
  ((uint8_t*)&sbus_data_.channels[10])[0]
    |= (((uint8_t*)&sbus_data_.channels[9])[1] & 0x1F) >> 3;
  ((uint8_t*)&sbus_data_.channels[10])[1] &= 0x07;

  ((uint8_t*)&sbus_data_.channels[9])[1] &= 0x07;

  // Channel 11 is [bytes(17)(3:0), bytes(16)(7:1)]
  ((uint8_t*)&sbus_data_.channels[11])[1] = rx_buffer[SBusByte(17)];
  ((uint8_t*)&sbus_data_.channels[11])[0] = rx_buffer[SBusByte(16)];
  sbus_data_.channels[11] >>= 1;
  ((uint8_t*)&sbus_data_.channels[11])[1] &= 0x07;

  for (uint8_t i = 0; i < 12; i++)
  {
    sbus_data_.channels[i] = 1024 - sbus_data_.channels[i];
  }

  // Make sure the stale bit is cleared.
  sbus_error_bits_ &= ~SBUS_ERROR_BIT_STALE;
}


// =============================================================================
// Private functions:

// This is a helper function that converts an byte index for the raw SBus
// message to the place it occurs in the buffer. This is necessary because the
// interrupt handler records the bytes in backwards order.
static inline uint8_t SBusByte(uint8_t n)
{
  return SBUS_MESSAGE_LENGTH - n;
}

// -----------------------------------------------------------------------------
static enum SBusSwitchState SBusChannelSwitchState(uint8_t channel)
{
  if (channel < 16)
  {
    return (sbus_data_.channels[channel] > -(SBUS_MAX / 2))
      + (sbus_data_.channels[channel] > (SBUS_MAX / 2));
  }
  else
  {
    return 2 * ((sbus_data_.binary & _BV(channel - 16)) != 0);
  }
}
