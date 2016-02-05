#ifndef SPI_H_
#define SPI_H_


#include <inttypes.h>


#define SPI_TX_BUFFER_LENGTH (150)

typedef void (*SPICallback)(void);


// =============================================================================
// Public functions:

void SPIInit(void);

// -----------------------------------------------------------------------------
uint8_t * RequestSPITxBuffer(void);

// -----------------------------------------------------------------------------
void SPIExchangeThenCallback(uint8_t tx_length, volatile uint8_t * rx_buffer,
  uint8_t rx_buffer_length, SPICallback callback_ptr);

// -----------------------------------------------------------------------------
void SPIRxThenCallback(volatile uint8_t * rx_buffer, uint8_t rx_buffer_length,
  SPICallback callback_ptr);

// -----------------------------------------------------------------------------
// This function initiates the transmission of the data in the Tx buffer.
void SPITxBuffer(uint8_t tx_length);

// -----------------------------------------------------------------------------
void SPITxBufferThenCallback(uint8_t tx_length, SPICallback callback_ptr);


#endif  // SPI_H_
