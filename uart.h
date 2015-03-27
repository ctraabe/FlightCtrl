#ifndef UART_H_
#define UART_H_


#include <inttypes.h>
#include <avr/pgmspace.h>


#define UARTPrintf(format, ...) UARTPrintf_P(PSTR(format), ##__VA_ARGS__)


// =============================================================================
// Public functions:

void UARTInit(void);

// -----------------------------------------------------------------------------
void UARTTxByte(uint8_t byte);

// -----------------------------------------------------------------------------
void UARTPrintf_P(const char *format, ...);


#endif  // UART_H_
