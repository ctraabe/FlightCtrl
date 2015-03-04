#include "sbus.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "timing.h"


// =============================================================================
// Private data:

#define USART1_BAUD (100000L)

// The following is not declared static so that it will be visible to sbus.S.
volatile int16_t sbus_last_byte_timestamp_ = 0;
volatile uint8_t sbus_data_ready_ = 0, rx_buffer_[48] = {0};
volatile uint8_t* volatile sbus_rx_ptr_ = &rx_buffer_[0];


// =============================================================================
// Public functions:

void SBusInit(void)
{
  // Set the baud rate.
  UBRR1 = F_CPU / 8 / USART1_BAUD - 1;
  // Set UART Double Speed (U2X).
  UCSR1A = (1 << U2X1);
  // Enable USART1 receiver and transmitter and interrupts.
  UCSR1B = (0 << RXCIE1)  // RX Complete Interrupt Enable
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
  uint8_t rx_byte = UDR1, rx_index;

  // Check for a gap of >1 ms since last reception.
  if (MillisSinceTimestamp(&sbus_last_byte_timestamp_) & 0xFFFE)
    rx_index = 0;

  if (!rx_index)
  {
    // Check for the start byte and that the appropriate amount of space has passed.
    if (rx_byte == 0xF8)
    {
      rx_index = 23;
      rx_buffer_[23] = rx_byte;
    }
  }
  else
  {
    if (!--rx_index)
    {
      if (rx_byte == 0x00 && !(rx_buffer_[0] & 0x0C))
        sbus_data_ready_ = 1;
    }
    else
    {
      rx_buffer_[rx_index] = rx_byte;
    }
  }
}