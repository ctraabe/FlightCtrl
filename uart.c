#include "uart.h"

#include <avr/io.h>


// =============================================================================
// Private data:

#define USART0_BAUD (57600)


// =============================================================================
// Public functions:

void UARTInit(void)
{
  // Set the baud rate.
  UBRR0 = F_CPU / 8 / USART0_BAUD - 1;
  // Set UART Double Speed (U2X).
  UCSR0A = (1 << U2X0);
  // Enable USART0 receiver and transmitter and interrupts.
  UCSR0B = (0 << RXCIE0)  // RX Complete Interrupt Enable
         | (0 << TXCIE0)  // TX Complete Interrupt Enable
         | (0 << UDRIE0)  // Data Register Empty Interrupt Enable
         | (1 << TXEN0)  // Transmitter Enable
         | (0 << RXEN0)  // Receiver Enable
         | (0 << UCSZ02);  // 9-bit Character Size Enable
  UCSR0C = (0 << UMSEL01) | (0 << UMSEL00)  // USART Mode (asynchronous)
         | (0 << UPM01) | (0 << UPM00)  // Parity Bit Mode (none)
         | (0 << USBS0)  // 2 Stop Bit Enable
         | (1 << UCSZ01) | (1 << UCSZ00);  // Character Size (8-bits)
}

// -----------------------------------------------------------------------------
void UARTTxByte(uint8_t byte)
{
  // TODO: should USART Data Register Empty Interrupt be used instead?
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = byte;
}


// =============================================================================
// Private functions:
