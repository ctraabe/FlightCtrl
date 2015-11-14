#include "spi.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "adc.h"
#include "led.h"
#include "mcu_pins.h"
#include "timing.h"


// =============================================================================
// Private data:

static uint8_t tx_buffer_[SPI_TX_BUFFER_LENGTH];
static volatile uint8_t logging_ = 0, tx_bytes_remaining_ = 0, * tx_ptr_ = 0;
static volatile uint16_t logging_timeout_ = 0;
static uint8_t tx_overflow_counter_ = 0;


// =============================================================================
// Private function declarations:

static inline void DeselectSlave(void);
static inline void SelectSlave(void);


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
// This function returns the address of the shared Tx buffer (tx_buffer_) if it
// is available of zero if not.
uint8_t * RequestSPITxBuffer(void)
{
  if (tx_bytes_remaining_)
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
  if (tx_length == 0) return;
  SelectSlave();
  SPDR = tx_buffer_[0];
  tx_ptr_ = &tx_buffer_[1];
  tx_bytes_remaining_ = tx_length - 1;
  SPCR |= _BV(SPIE);  // Enable the SPI transmission complete interrupt.
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
// Transmission (byte) complete interrupt
ISR(SPI_STC_vect)
{
  DeselectSlave();
  if (!tx_bytes_remaining_)
  {
    SPCR &= ~_BV(SPIE);  // Disable this interrupt
  }
  else
  {
    --tx_bytes_remaining_;
    SelectSlave();
    SPDR = *(tx_ptr_++);
  }

  uint8_t rx_byte = SPDR;
  if (rx_byte == 0xCC)
  {
    logging_timeout_ = GetTimestampMillisFromNow(100);
    logging_ = 1;
    RedLEDOn();
  }
  else if (rx_byte == 0x33)
  {
    logging_ = 0;
    RedLEDOff();
  }
}
