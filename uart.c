#include "uart.h"

#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/io.h>


// =============================================================================
// Private data:

#define USART0_BAUD (57600)
#define RX_BUFFER_LENGTH (32)

// volatile uint8_t rx_byte_ = 0, rx_length_ = 0, rx_buffer_[RX_BUFFER_LENGTH];
volatile uint8_t tx_source_len_ = 0, *tx_source_ptr_ = 0;


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

// -----------------------------------------------------------------------------
void UARTTxBytes(uint8_t *tx_source_ptr, uint8_t tx_source_len)
{
  if (!tx_source_len_ && (UCSR0A & _BV(UDRE0))) {
    tx_source_ptr_ = tx_source_ptr;
    tx_source_len_ = tx_source_len;
    UDR0 = *tx_source_ptr;
    UCSR0B |= _BV(UDRIE0);  // Enable the USART0 data register empty interrupt.
  }
}

// -----------------------------------------------------------------------------
// This function acts like printf, but puts the result on the UART stream. It
// also adds the end-of-line characters and checks that the character buffer is
// not exceeded. Note that this function is blocking.
void UARTPrintf_P(const char *format, ...)
{
  // TODO: never when motors are running...
  static char ascii[103];  // 100 chars + 2 newline chars + null terminator

  va_list arglist;
  va_start(arglist, format);
  int length = vsnprintf_P(ascii, 101, format, arglist);
  va_end(arglist);

  if (length < 101)
    sprintf_P(&ascii[length], PSTR("\n\r"));
  else
    sprintf_P(&ascii[80], PSTR("... MESSAGE TOO LONG\n\r"));

  char *pointer = &ascii[0];
  while (*pointer) UARTTxByte(*pointer++);
}


// =============================================================================
// Private functions:

// This function is called upon the "USART0 data register empty" interrupt,
// indicating that the transmitter is ready to load another byte.
ISR(USART0_UDRE_vect)
{
  if (--tx_source_len_) {
    UDR0 = *(++tx_source_ptr_);
  } else {
    // This interrupt is triggered whenever the data register is empty, so must
    // be disabled after the final transmission.
    UCSR0B &= ~_BV(UDRIE0);
  }
}
