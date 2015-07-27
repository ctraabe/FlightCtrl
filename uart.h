#ifndef UART_H_
#define UART_H_


#define RX_BUFFER_LENGTH_POWER_OF_2 (6)  // 2^6 = 64
#define RX_BUFFER_LENGTH (1 << RX_BUFFER_LENGTH_POWER_OF_2)

#ifndef __ASSEMBLER__


#include <inttypes.h>
#include <avr/pgmspace.h>


#define UARTPrintf(format, ...) UARTPrintf_P(PSTR(format), ##__VA_ARGS__)


// =============================================================================
// Public functions:

void UARTInit(void);

// -----------------------------------------------------------------------------
void UARTTxByte(uint8_t byte);

// -----------------------------------------------------------------------------
void UARTTxBytes(uint8_t *tx_source_ptr, uint8_t tx_source_len);

// -----------------------------------------------------------------------------
void UARTPrintf_P(const char *format, ...);


#endif  // __ASSEMBLER__

#endif  // UART_H_
