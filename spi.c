#include "spi.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "mcu_pins.h"
// TODO: Remove
#include "adc.h"
#include "timing.h"


// =============================================================================
// Private data:

#define TX_BUFFER_LENGTH (20)

static uint8_t tx_buffer_[TX_BUFFER_LENGTH];
static volatile uint8_t tx_bytes_remaining_ = 0, *tx_ptr_ = 0;


// =============================================================================
// Public functions:

void SPIInit(void)
{
  // Set MOSI and SCK pins to output.
  SPI_DDR = SPI_MOSI_PIN | SPI_SCK_PIN;
  SPCR = (0 << SPIE)  // SPI Interrupt (not) Enable
       | (1 << SPE)  // SPI Enable
       | (0 << DORD)  // Data Order
       | (1 << MSTR)  // Master Select
       | (0 << CPOL)  // Clock Polarity
       | (0 << CPHA)  // Clock Phase
       | (1 << SPR0) | (1 << SPR0);  // Set clock to F_CPU / 16
}

// -----------------------------------------------------------------------------
// This function initiates the transmission of the data in the Tx buffer.
void SPITxBuffer(uint8_t tx_length)
{
  if (tx_length == 0) return;
  tx_ptr_ = &tx_buffer_[0];
  tx_bytes_remaining_ = tx_length;
  SPCR |= _BV(SPIE);  // Enable the SPI transmission complete interrupt.
}

// -----------------------------------------------------------------------------
void SPITxSensorData(void)
{
  struct SensorData {
    uint16_t timestamp;
    int16_t accelerometer_sum[3];
    int16_t gyro_sum[3];
    uint16_t biased_pressure;
    uint16_t battery_voltage;
  } __attribute__((packed));

  struct SensorData * sensor_data_ptr = (struct SensorData *)tx_buffer_;

  sensor_data_ptr->timestamp = GetTimestamp();
  sensor_data_ptr->accelerometer_sum[0] = AccelerometerSum(X_BODY_AXIS);
  sensor_data_ptr->accelerometer_sum[1] = AccelerometerSum(Y_BODY_AXIS);
  sensor_data_ptr->accelerometer_sum[2] = AccelerometerSum(Z_BODY_AXIS);
  sensor_data_ptr->gyro_sum[0] = GyroSum(X_BODY_AXIS);
  sensor_data_ptr->gyro_sum[1] = GyroSum(Y_BODY_AXIS);
  sensor_data_ptr->gyro_sum[2] = GyroSum(Z_BODY_AXIS);
  sensor_data_ptr->biased_pressure = BiasedPressure();
  sensor_data_ptr->battery_voltage = BatteryVoltage();

  tx_ptr_ = (uint8_t *)sensor_data_ptr;
  tx_bytes_remaining_ = sizeof(struct SensorData);
  SPCR |= _BV(SPIE);  // Enable the SPI transmission complete interrupt.
}


// =============================================================================
// Private functions:

// Transmission (byte) complete interrupt
ISR(SPI_STC_vect)
{
  if (tx_bytes_remaining_)
  {
    SPDR = *(tx_ptr_++);
    tx_bytes_remaining_--;
  }
  else
  {
    SPCR &= ~_BV(SPIE);  // Disable this interrupt
  }
}
