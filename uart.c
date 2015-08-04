#include "uart.h"

#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "mcu_pins.h"
#include "mk_serial_protocol.h"
#include "mk_serial_tx.h"


// =============================================================================
// Private data:

#define USART0_BAUD (57600)

volatile uint8_t rx_buffer_head_ = 0, rx_buffer_[RX_BUFFER_LENGTH];
static volatile uint8_t tx_bytes_remaining_ = 0, *tx_ptr_ = 0;
static uint8_t data_buffer_[DATA_BUFFER_LENGTH], tx_buffer_[TX_BUFFER_LENGTH];
static uint8_t tx_overflow_counter_ = 0;


// =============================================================================
// Public functions:

void UARTInit(void)
{
  // Pull up Rx pin.
  UART_PORT |= UART_RX_PIN;
  // Set the baud rate.
  UBRR0 = F_CPU / 8 / USART0_BAUD - 1;
  // Set UART Double Speed (U2X).
  UCSR0A = (1 << U2X0);
  // Enable USART0 receiver and transmitter and interrupts.
  UCSR0B = (1 << RXCIE0)  // RX Complete Interrupt Enable
         | (0 << TXCIE0)  // TX Complete Interrupt Enable
         | (0 << UDRIE0)  // Data Register Empty Interrupt Enable
         | (1 << TXEN0)  // Transmitter Enable
         | (1 << RXEN0)  // Receiver Enable
         | (0 << UCSZ02);  // 9-bit Character Size Enable
  UCSR0C = (0 << UMSEL01) | (0 << UMSEL00)  // USART Mode (asynchronous)
         | (0 << UPM01) | (0 << UPM00)  // Parity Bit Mode (none)
         | (0 << USBS0)  // 2 Stop Bit Enable
         | (1 << UCSZ01) | (1 << UCSZ00);  // Character Size (8-bits)
}

// -----------------------------------------------------------------------------
void ProcessIncomingUART(void)
{
  static uint8_t rx_buffer_tail = 0;
  static enum UARTRxMode mode = UART_RX_MODE_IDLE;

  while (rx_buffer_tail != rx_buffer_head_)
  {
    rx_buffer_tail = (rx_buffer_tail + 1) % RX_BUFFER_LENGTH;

    // TODO: Support other protocols
    if (mode != UART_RX_MODE_IDLE)
      mode = MKSerialRx(rx_buffer_[rx_buffer_tail], data_buffer_);
    else if (rx_buffer_[rx_buffer_tail] == '#')
      mode = UART_RX_MODE_MK_ONGOING;
  }
}

// -----------------------------------------------------------------------------
void SendUART(void)
{
  // TODO: add other transmit protocols here.
  SendMKSerial();
}

// -----------------------------------------------------------------------------
uint8_t * UARTTxBuffer(void)
{
  if (tx_bytes_remaining_)
  {
    tx_overflow_counter_++;
    return 0;
  }
  return tx_buffer_;
}

// -----------------------------------------------------------------------------
void UARTTxByte(uint8_t byte)
{
  // TODO: should USART Data Register Empty Interrupt be used instead?
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = byte;
}

// -----------------------------------------------------------------------------
void UARTTx(uint8_t tx_length)
{
  if (tx_length == 0) return;
  tx_ptr_ = &tx_buffer_[0];
  tx_bytes_remaining_ = tx_length;

  // Go ahead and send a byte if the transmitter is ready.
  if (UCSR0A & _BV(UDRE0))
  {
    UDR0 = *(tx_ptr_++);
    tx_bytes_remaining_--;
  }

  UCSR0B |= _BV(UDRIE0);  // Enable the USART0 data register empty interrupt.
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
  if (tx_bytes_remaining_)
  {
    UDR0 = *(tx_ptr_++);
    tx_bytes_remaining_--;
  }
  else
  {
    UCSR0B &= ~_BV(UDRIE0);  // Disable this interrupt
  }
}

// -----------------------------------------------------------------------------
ISR(USART0_RX_vect)
{
  rx_buffer_head_ = (rx_buffer_head_ + 1) % RX_BUFFER_LENGTH;
  rx_buffer_[rx_buffer_head_] = UDR0;
}
