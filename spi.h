#ifndef SPI_H_
#define SPI_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

void SPIInit(void);

// -----------------------------------------------------------------------------
// This function initiates the transmission of the data in the Tx buffer.
void SPITxBuffer(uint8_t tx_length);

// -----------------------------------------------------------------------------
void SPITxSensorData(void);


#endif  // SPI_H_
