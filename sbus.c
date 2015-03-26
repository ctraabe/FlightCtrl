#include "sbus.h"

#include <avr/interrupt.h>

#include "led.h"


// =============================================================================
// Private data:

#define USART1_BAUD (100000L)
#define SBUS_NO_NEW_DATA (-1)
#define SBUS_SIGNAL_LOST_BIT (2)

// The following is not declared static so that it will be visible to sbus.S.
volatile uint8_t sbus_rx_buffer_[2][SBUS_RX_BUFFER_LENGTH];  // double buffer
volatile int8_t sbus_data_ready_ = SBUS_NO_NEW_DATA;

struct SBusData
{
  int16_t channels[12];
  uint8_t binary;
  uint8_t valid;
  int16_t timestamp;
} __attribute__((packed)) sbus_data_;


// =============================================================================
// Private function declarations:

inline uint8_t SBusByte(uint8_t);


// =============================================================================
// Accessors:

uint8_t SBusBinary(uint8_t n)
{
  return !(sbus_data_.binary & (1 << n));
}

// -----------------------------------------------------------------------------
int16_t SBusChannel(uint8_t n)
{
  return sbus_data_.channels[n];
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
}

// -----------------------------------------------------------------------------
void ProcessSBus(void)
{
  // TODO: check for stale data
  if (sbus_data_ready_ == SBUS_NO_NEW_DATA) return;

  // Make a local (non-volatile) pointer to the appropriate side of
  // sbus_rx_buffer_.
  uint8_t* rx_buffer = (uint8_t*)&sbus_rx_buffer_[sbus_data_ready_][0];

  // Reset the sbus_data_ready_ indicator.
  sbus_data_ready_ = SBUS_NO_NEW_DATA;

  // Check that the received data is valid.
  if ((rx_buffer[SBusByte(24)] != SBUS_END_BYTE)
    || (rx_buffer[SBusByte(23)] & (1 << SBUS_SIGNAL_LOST_BIT)))
  {
    return;
  }

  // Record the timestamp
  ((uint8_t*)&sbus_data_.timestamp)[0] = rx_buffer[SBUS_RX_BUFFER_LENGTH - 2];
  ((uint8_t*)&sbus_data_.timestamp)[1] = rx_buffer[SBUS_RX_BUFFER_LENGTH - 1];

  sbus_data_.binary = rx_buffer[SBusByte(23)];
  // PORTB ^= _BV(PORTB0);

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
}


// =============================================================================
// Private functions:

// This is a helper function that converts an byte index for the raw SBus
// message to the place it occurs in the buffer. This is necessary because the
// interrupt handler records the bytes in backwards order.
inline uint8_t SBusByte(uint8_t n)
{
  return SBUS_MESSAGE_LENGTH - n;
}
