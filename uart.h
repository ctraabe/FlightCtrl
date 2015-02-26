#ifndef UART_H_
#define UART_H_

#include <inttypes.h>
#include <avr/io.h>


// ============================================================================+
// Global data:
//
// Warning: this data is made global so that it can be included in inline
// functions to speed execution of often-used functions. This global data should
// not be set outside of this program unit.


// ============================================================================+
// Inline functions:

inline void UARTTxByte(uint8_t byte)
{
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = byte;
}


// ============================================================================+
// Public functions:
void UARTInit(void);

#endif // UART_H_
