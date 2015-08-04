#ifndef UART_H_
#define UART_H_


#define RX_BUFFER_LENGTH_POWER_OF_2 (6)  // 2^6 = 64
#define RX_BUFFER_LENGTH (1 << RX_BUFFER_LENGTH_POWER_OF_2)

#ifndef __ASSEMBLER__


#include <inttypes.h>
#include <avr/pgmspace.h>


#define DATA_BUFFER_LENGTH (70)
#define TX_BUFFER_LENGTH (70)
#define UARTPrintf(format, ...) UARTPrintf_P(PSTR(format), ##__VA_ARGS__)

enum UARTRxMode {
  UART_RX_MODE_IDLE = 0,
  UART_RX_MODE_MK_ONGOING,
};


// =============================================================================
// Public functions:

void UARTInit(void);

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler. Each byte is passed to the
// appropriate Rx handler, which may place it into the temporary data buffer
// (data_buffer_).
void ProcessIncomingUART(void);

// -----------------------------------------------------------------------------
// This function returns the address of the shared Tx buffer (tx_buffer_) if it
// is available of zero if not.
uint8_t * RequestUARTTxBuffer(void);

// -----------------------------------------------------------------------------
// This function calls handlers for pending data transmission requests.
void SendPendingUART(void);

// -----------------------------------------------------------------------------
// This function initiates the transmission of the data in the Tx buffer.
void UARTTxBuffer(uint8_t tx_length);

// -----------------------------------------------------------------------------
// This function immediately transmits a byte and blocks computation until
// transmission is commenced.
void UARTTxByte(uint8_t byte);

// -----------------------------------------------------------------------------
// This function mimics printf, but puts the result on the UART stream. It also
// adds the end-of-line characters and checks that the character buffer is not
// exceeded. Note that this function is slow and blocking.
void UARTPrintf_P(const char *format, ...);


#endif  // __ASSEMBLER__

#endif  // UART_H_
