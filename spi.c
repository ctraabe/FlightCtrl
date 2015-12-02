#include "spi.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "adc.h"
#include "mcu_pins.h"
#include "timing.h"


// =============================================================================
// Private data:

static volatile uint8_t rx_bytes_remaining_ = 0, * volatile rx_ptr_ = 0;
static volatile uint8_t tx_bytes_remaining_ = 0;
static const uint8_t * volatile tx_ptr_ = 0;

static uint8_t tx_buffer_[SPI_TX_BUFFER_LENGTH], tx_overflow_counter_ = 0;
static SPICallback callback_ptr_ = 0;
static volatile uint8_t temp = 0;

// =============================================================================
// Private function declarations:

static inline void DeselectSlave(void);
static inline void SelectSlave(void);
static inline void SPITxByte(uint8_t byte);


// =============================================================================
// Public functions:

void SPIInit(void)
{
  // Set MOSI and SCK pins to output.
  SPI_DDR |= SPI_MOSI_PIN | SPI_SCK_PIN;
  SPCR = (0 << SPIE)  // SPI Interrupt (not) Enable
       | (1 << SPE)  // SPI Enable
       | (0 << DORD)  // Data Order
       | (1 << MSTR)  // Master Select
       | (0 << CPOL)  // Clock Polarity
       | (0 << CPHA)  // Clock Phase
       | (0 << SPR1) | (1 << SPR0);  // Set clock to F_CPU / 16

  // Set the slave select pin to output and deselect slave.
  SPI_SS_DDR |= SPI_SS_PIN;
  DeselectSlave();  // Deselect the slave
}

// -----------------------------------------------------------------------------
uint8_t SPIRxThenCallback(uint8_t * rx_buffer, uint8_t rx_buffer_length,
  SPICallback callback_ptr)
{
  if (tx_bytes_remaining_ != 0 || rx_bytes_remaining_ != 0 || rx_buffer == 0
    || rx_buffer_length == 0) return 0;
  rx_ptr_ = rx_buffer;
  rx_bytes_remaining_ = rx_buffer_length;
  callback_ptr_ = callback_ptr;
  SPITxByte(0xFF);  // Start transmission with a dummy byte
  return 1;  // Success
}

// -----------------------------------------------------------------------------
// This function returns the address of the shared Tx buffer (tx_buffer_) if it
// is available of zero if not.
uint8_t * RequestSPITxBuffer(void)
{
  if (tx_bytes_remaining_ != 0)
  {
    tx_overflow_counter_++;
    return 0;
  }
  return tx_buffer_;
}

// -----------------------------------------------------------------------------
// This function initiates the transmission of the data in the Tx buffer.
void SPITxBuffer(uint8_t tx_length)
{
  if (tx_bytes_remaining_ != 0 || rx_bytes_remaining_ != 0 || tx_length == 0
    || tx_length > SPI_TX_BUFFER_LENGTH) return;
  tx_ptr_ = &tx_buffer_[0];  // Set the transmit pointer to the next byte
  SPITxByte(*tx_ptr_);  // Start transmission with the first byte
  tx_bytes_remaining_ = tx_length - 1;
}


// =============================================================================
// Private functions:

static inline void DeselectSlave(void)
{
  SPI_SS_PORT |= SPI_SS_PIN;
}

// -----------------------------------------------------------------------------
static inline void SelectSlave(void)
{
  SPI_SS_PORT &= ~SPI_SS_PIN;
}

// -----------------------------------------------------------------------------
static inline void SPITxByte(uint8_t byte)
{
  SelectSlave();
  SPDR = byte;
  SPCR |= _BV(SPIE);  // Enable the SPI transmission complete interrupt
}

// -----------------------------------------------------------------------------
// Transmission (byte) complete interrupt. Note, this is a very high frequency
// interrupt (around 150kHz), so SPI transmission should be avoided until after
// high-priority processing is finished. Also, make sure that any callback is
// very brief.
ISR(SPI_STC_vect)
{
  DeselectSlave();

  if (rx_bytes_remaining_ != 0)
  {
    *rx_ptr_++ = SPDR;
    if (--rx_bytes_remaining_ == 0 && callback_ptr_) (*callback_ptr_)();
  }
  else
  {
    temp = SPDR;  // Empty the RX buffer
  }

  if (tx_bytes_remaining_ != 0)
  {
    --tx_bytes_remaining_;
    SelectSlave();
    SPDR = *(++tx_ptr_);
  }
  else if (rx_bytes_remaining_ != 0)
  {
    // Send a dummy byte to set the clock for reception.
    SelectSlave();
    SPDR = 0xFF;
  }
  else
  {
    SPCR &= ~_BV(SPIE);  // Disable this interrupt
  }
}
