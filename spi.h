#ifndef SPI_H_
#define SPI_H_


#include <inttypes.h>


#define SPI_TX_BUFFER_LENGTH (64)

typedef void (*SPICallback)(void);


// =============================================================================
// Public functions:

void SPIInit(void);

// -----------------------------------------------------------------------------
uint8_t SPIRxThenCallback(uint8_t * rx_buffer, uint8_t rx_buffer_length,
  SPICallback callback_ptr);

// -----------------------------------------------------------------------------
uint8_t * RequestSPITxBuffer(void);

// -----------------------------------------------------------------------------
// This function initiates the transmission of the data in the Tx buffer.
void SPITxBuffer(uint8_t tx_length);


#endif  // SPI_H_
