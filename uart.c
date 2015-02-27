#include "uart.h"

#include <avr/interrupt.h>


// =============================================================================
// Private data:

#define USART0_BAUD (57600)


// =============================================================================
// Public functions:

inline void UARTInit(void)
{
  // Set the baud rate.
  UBRR0 = F_CPU / 8 / USART0_BAUD - 1;
  // Set UART Double Speed (U2X).
  UCSR0A = (1 << U2X0);
  // Enable USART0 receiver and transmitter and interrupts.
  UCSR0B = (1 << TXEN0)  // Enable the USART0 transmitter.
         | (0 << RXEN0)  // Enable the USART0 receiver.
         | (0 << TXCIE0)  // Enable the Transmit Complete interrupt.
         | (0 << RXCIE0);  // Enable the Receive Complete interrupt.
}

// -----------------------------------------------------------------------------
inline void UARTTxByte(uint8_t byte)
{
  // TODO: should USART Data Register Empty Interrupt be used instead?
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = byte;
}


// =============================================================================
// Private functions:
