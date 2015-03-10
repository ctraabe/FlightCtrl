#ifndef UART_H_
#define UART_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

void UARTInit(void);

// -----------------------------------------------------------------------------
void UARTTxByte(uint8_t byte);


#endif  // UART_H_
