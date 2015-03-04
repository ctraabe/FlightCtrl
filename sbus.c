#include "sbus.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "timing.h"


// =============================================================================
// Private data:

#define USART1_BAUD (100000L)
#define SBUS_MESSAGE_LENGTH (23)
#define SBUS_START_BYTE (0x0F)
#define SBUS_END_BYTE (0x00)

// The following is not declared static so that it will be visible to sbus.S.
volatile int16_t sbus_last_byte_timestamp_ = 0;
volatile int16_t sbus_timestamp_ = 0;
volatile uint8_t sbus_rx_buffer_[2][SBUS_MESSAGE_LENGTH], sbus_buffer_ch_ = 0;
volatile uint8_t* volatile sbus_data_ptr_ = 0;


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

// To be implemented in assembly
ISR(USART1_RX_vect)
{
  uint8_t rx_byte = UDR1;
  static uint8_t rx_counter = 0;

  // Check for a gap of >1 ms since last reception.
  if (MillisSinceTimestamp(&sbus_last_byte_timestamp_) & 0xFFFE)
    rx_counter = 0;

  if (!rx_counter)
  {
    // Check for the start byte.
    if (rx_byte == SBUS_START_BYTE) rx_counter = SBUS_MESSAGE_LENGTH + 1;
  }
  else
  {
    if (--rx_counter)
    {
      sbus_rx_buffer_[sbus_buffer_ch_][rx_counter-1] = rx_byte;
    }
    else
    {
      if (rx_byte == SBUS_END_BYTE && !(sbus_rx_buffer_[sbus_buffer_ch_][0] & 0x0C))
      {
        sbus_data_ptr_ = &sbus_rx_buffer_[sbus_buffer_ch_][0];
        sbus_buffer_ch_ ^= 0x01;
        sbus_timestamp_ = GetTimestamp();
      }
    }
  }
}