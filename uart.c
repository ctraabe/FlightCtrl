#include "uart.h"

#include <avr/interrupt.h>

// ============================================================================+
// Private data:

#define USART0_BAUD (57600)


// ============================================================================+
// Public functions:

void UARTInit(void)
{
  // Set the baud rate.
  UBRR0 = F_CPU / 8 / USART0_BAUD - 1;
  // Set UART Double Speed (U2X).
  UCSR0A |= (1 << U2X0);
  // Enable USART0 receiver and transmitter and interrupts.
  UCSR0B = (1 << TXEN0)  // Enable the USART0 transmitter.
      | (1 << RXEN0)     // Enable the USART0 receiver.
      | (1 << TXCIE0)    // Enable the Transmit Complete interrupt.
      | (1 << RXCIE0);   // Enable the Receive Complete interrupt.
}


// ============================================================================+
// Private functions:

// -----------------------------------------------------------------------------
// This function is called upon the "USART0 Tx complete" interrupt.
ISR(USART0_TX_vect)
{
}

// -----------------------------------------------------------------------------
// This function is called upon the "USART0 Rx complete" interrupt.
ISR(USART0_RX_vect)
{
}