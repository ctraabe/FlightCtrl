// Information:
//
// UART stands for Universal Asynchronous Receiver/Transmitter. A UART uses 2
// wires for data transmission: one for transmitting, and one for receiving.
// Data transfers may be started at any time, but must occur at a pre-agreed
// rate, called the BAUD rate. Transmit and receive can occur simultaneously
// (i.e. one does not impact the other). When no data is being transferred, the
// output is set to the high state (1). A single low bit (0) marks the start of
// a new data packet. This UART is set to the following:
//
//   - 57600 BAUD (57.6 kbits / second)
//   - 8 bits of data follow the single start bit
//   - data packets are closed with a single high bit and without a parity bit
//
// Additionally, this file sets up a buffering scheme for minimal-impact,
// interrupt-based serial transmission and reception.
//
// Incoming bytes are immediately placed into a small ring buffer (rx_buffer_)
// by the Rx interrupt handler. The function ProcessIncomingUART() passes bytes
// from the ring buffer to the appropriate Rx handler. A larger shared data
// buffer (data_buffer_) is provided for temporary storage for the Rx handlers.
// The data in the shared data buffer must be processed immediately after the
// final byte in a message is read form the ring buffer so that the data buffer
// may be used in handling the next message.
//
// The function SendPendingUART() invokes functions that handle data Tx
// requests. A shared Tx buffer (tx_buffer_) is provided for interrupt-based
// transmission. The Tx buffer must be requested via the function
// RequestTxBuffer(), since the Tx buffer should not modified during an ongoing
// transmission. If access to the buffer is granted, the outgoing message should
// be written to the Tx buffer. The function UARTTxBuffer() initiates the
// interrupt-driven transmission of the buffer.
//
// Two other BLOCKING functions are provided to immediately send a single byte
// (UARTTxByte) and to mimic the standard printf function (UARTPrintf). These
// functions are BLOCKING, meaning that non-interrupt computation is blocked
// until transmission is complete. Therefore, these functions should not be
// called when the motors are running.

#include "uart.h"

#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "mcu_pins.h"
#include "mk_serial_protocol.h"
#include "mk_serial_tx.h"
#include "state.h"
#include "timing.h"
#include "ut_serial_protocol.h"


// =============================================================================
// Private data:

#define USART0_BAUD (57600)

static volatile uint8_t rx_buffer_head_ = 0, rx_buffer_[UART_RX_BUFFER_LENGTH];
static volatile uint8_t tx_bytes_remaining_ = 0;
static const uint8_t * volatile tx_ptr_ = 0;
static uint8_t data_buffer_[UART_DATA_BUFFER_LENGTH];
static uint8_t tx_buffer_[UART_TX_BUFFER_LENGTH];
static uint8_t tx_overflow_counter_ = 0;


// =============================================================================
// Private function declarations:

static void Printf(const char *format, va_list arglist);


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
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler. Each byte is passed to the
// appropriate Rx handler, which may place it into the temporary data buffer
// (data_buffer_).
void ProcessIncomingUART(void)
{
  static uint8_t rx_buffer_tail = 0;
  static enum UARTRxMode mode = UART_RX_MODE_IDLE;

  while (rx_buffer_tail != rx_buffer_head_)
  {
    // Move the ring buffer tail forward.
    rx_buffer_tail = (rx_buffer_tail + 1) % UART_RX_BUFFER_LENGTH;

    // Add other Rx protocols here.
    switch (mode)
    {
      case UART_RX_MODE_UT_ONGOING:
        mode = UTSerialRx(rx_buffer_[rx_buffer_tail], data_buffer_);
        break;
      case UART_RX_MODE_MK_ONGOING:
        mode = MKSerialRx(rx_buffer_[rx_buffer_tail], data_buffer_);
        break;
      default:
        if (rx_buffer_[rx_buffer_tail] == UT_START_CHARACTER)
          mode = UART_RX_MODE_UT_ONGOING;
        else if (rx_buffer_[rx_buffer_tail] == MK_START_CHARACTER)
          mode = UART_RX_MODE_MK_ONGOING;
    }
  }
}

// -----------------------------------------------------------------------------
// This function returns the address of the shared Tx buffer (tx_buffer_) if it
// is available of zero if not.
uint8_t * RequestUARTTxBuffer(void)
{
  if (tx_bytes_remaining_ != 0)
  {
    tx_overflow_counter_++;
    return 0;
  }
  return tx_buffer_;
}

// -----------------------------------------------------------------------------
// This function calls handlers for pending data transmission requests.
void SendPendingUART(void)
{
  // Add other Tx protocols here.
  SendPendingMKSerial();
}

// -----------------------------------------------------------------------------
// This function initiates the transmission of the data in the Tx buffer.
void UARTTxBuffer(uint8_t tx_length)
{
  if (tx_bytes_remaining_ != 0 || tx_length == 0
    || tx_length > UART_TX_BUFFER_LENGTH) return;
  tx_ptr_ = &tx_buffer_[0];
  tx_bytes_remaining_ = tx_length;
  UCSR0B |= _BV(UDRIE0);  // Enable the USART0 data register empty interrupt.
}

// -----------------------------------------------------------------------------
// This function immediately transmits a byte and blocks computation until
// transmission is commenced.
void UARTTxByte(uint8_t byte)
{
  // Never allow blocking when motors are running.
  if (!MotorsInhibited()) return;
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = byte;
}

// -----------------------------------------------------------------------------
uint32_t UARTWaitUntilCompletion(uint32_t time_limit_ms)
{
  uint32_t timeout = GetTimestampMillisFromNow(time_limit_ms);
  while ((tx_bytes_remaining_ != 0) && !TimestampInPast(timeout)) continue;
  return TimestampInPast(timeout);
}

// -----------------------------------------------------------------------------
// This function mimics printf, but puts the result on the UART stream. It also
// adds the end-of-line characters and checks that the character buffer is not
// exceeded. This version blocks program execution until UART is available and
// then further blocks execution until the transmission has competed.
void UARTPrintf_P(const char *format, ...)
{
  // Never allow blocking when motors are running.
  if (!MotorsInhibited()) return;
  UARTWaitUntilCompletion(500);

  va_list arglist;
  va_start(arglist, format);
  Printf(format, arglist);
  va_end(arglist);
  UARTWaitUntilCompletion(500);
}

// -----------------------------------------------------------------------------
// This function mimics printf, but puts the result on the UART stream. It also
// adds the end-of-line characters and checks that the character buffer is not
// exceeded. This version attempts to get the UART Tx buffer and then initiates
// an interrupt-bases transmission. This function is non-blocking, but may fail
// to get access to the UART Tx buffer.
void UARTPrintfSafe_P(const char *format, ...)
{
  va_list arglist;
  va_start(arglist, format);
  Printf(format, arglist);
  va_end(arglist);
}


// =============================================================================
// Private functions:

// This function mimics printf, but puts the result on the UART stream. It also
// adds the end-of-line characters and checks that the character buffer is not
// exceeded.
static void Printf(const char *format, va_list arglist)
{
  // Buffer requirement: 100 chars + 2 newline chars + 1 null terminator
  _Static_assert(UART_TX_BUFFER_LENGTH >= 103,
    "UART buffer not large enough for UARTPrintf");

  uint8_t * ascii = RequestUARTTxBuffer();
  if (!ascii) return;

  int length = vsnprintf_P((char *)ascii, 101, format, arglist);

  if (length < 101)
  {
    sprintf_P((char *)&ascii[length], PSTR("\n\r"));
    length += 2;
  }
  else
  {
    sprintf_P((char *)&ascii[80], PSTR("... MESSAGE TOO LONG\n\r"));
    length = 103;
  }

  UARTTxBuffer(length);
}

// -----------------------------------------------------------------------------
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
  rx_buffer_head_ = (rx_buffer_head_ + 1) % UART_RX_BUFFER_LENGTH;
  rx_buffer_[rx_buffer_head_] = UDR0;
}
