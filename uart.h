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
void ProcessIncomingUART(void);

// -----------------------------------------------------------------------------
void SendUART(void);

// -----------------------------------------------------------------------------
uint8_t * UARTTxBuffer(void);

// -----------------------------------------------------------------------------
void UARTTxByte(uint8_t byte);

// -----------------------------------------------------------------------------
void UARTTx(uint8_t tx_length);

// -----------------------------------------------------------------------------
void UARTPrintf_P(const char *format, ...);


#endif  // __ASSEMBLER__

#endif  // UART_H_
