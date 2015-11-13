#include "spi.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "adc.h"
#include "led.h"
#include "mcu_pins.h"
#include "timing.h"


// =============================================================================
// Private data:

#define TX_BUFFER_LENGTH (32)
#define SPI_START_BYTE (0xFE)

static uint8_t tx_buffer_[TX_BUFFER_LENGTH];
static volatile uint8_t logging_ = 0, tx_bytes_remaining_ = 0, * tx_ptr_ = 0;
static volatile uint16_t logging_timeout_ = 0;


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

// -----------------------------------------------------------------------------
void SPITxSensorData(void)
{
  static uint8_t counter_128_hz = 0;
  if (logging_ == 0 || TimestampInPast(logging_timeout_))
  {
    counter_128_hz = 0xFF;
    logging_ = 0;
  }
  else
  {
    counter_128_hz = (counter_128_hz + 1) & 0x7F;
  }

  // Send the counter out over UART1.
  UDR1 = counter_128_hz;

  tx_buffer_[0] = SPI_START_BYTE;

  struct SensorData {
    int16_t accelerometer_sum[3];
    int16_t gyro_sum[3];
    uint16_t biased_pressure;
    uint8_t counter_128_hz;
  } __attribute__((packed));

  struct SensorData * sensor_data_ptr = (struct SensorData *)&tx_buffer_[1];

  sensor_data_ptr->accelerometer_sum[0] = AccelerometerSum(X_BODY_AXIS);
  sensor_data_ptr->accelerometer_sum[1] = AccelerometerSum(Y_BODY_AXIS);
  sensor_data_ptr->accelerometer_sum[2] = AccelerometerSum(Z_BODY_AXIS);
  sensor_data_ptr->gyro_sum[0] = GyroSum(X_BODY_AXIS);
  sensor_data_ptr->gyro_sum[1] = GyroSum(Y_BODY_AXIS);
  sensor_data_ptr->gyro_sum[2] = GyroSum(Z_BODY_AXIS);
  sensor_data_ptr->biased_pressure = BiasedPressureSum();
  sensor_data_ptr->counter_128_hz = counter_128_hz;

  // Add 4 trailing zeros to force STR91x SPI Rx interrupt.
  tx_buffer_[sizeof(struct SensorData) + 1] = 0x00;
  tx_buffer_[sizeof(struct SensorData) + 2] = 0x00;
  tx_buffer_[sizeof(struct SensorData) + 3] = 0x00;
  tx_buffer_[sizeof(struct SensorData) + 4] = 0x00;

  SPITxBuffer(sizeof(struct SensorData) + 5);
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
