// Information:
//
// UART stands for Universal Asynchronous Receiver/Transmitter. A UART uses 2
// wires for data transmission: one for transmitting, and one for receiving.
// Data transfers may be started at any time, but must occur at a pre-agreed
// rate, called the BAUD rate. Transmit and receive can occur simultaneously
// (i.e. one does not impact the other). When no data is being transferred, the
// output is set to the high state (1). A single low bit (0) marks the start of
// a new data packet. This UART has been set to the following:
//
//   - 57600 BAUD (57.6 kbits / second)
//   - 8 bits of data follow the single start bit
//   - data packets are closed with a single high bit and without a parity bit
//
// With the above settings, each byte of data requires a 10-bit packet.  This
// means a maximum data rate of 5.76 kBytes / second. However, MikroKopter uses
// an additional protocol to manage the data transfer. In this protocol,
// multiple bytes of data can be wrapped into a single frame with some
// identifying information. The identifying information includes the following
// at the start of the frame:
//
//   - the start character "#"
//   - an address
//   - a command identifier
//
// and the following at the end of the frame:
//
//   - 2 bytes containing the value for the Cyclic Redundancy Check (CRC)
//   - the end character "\r"
//
// Finally, to guarantee identification of the start and end characters, data is
// separated into 3-byte chunks and each chunk is re-encoded into 4-bytes. Each
// of the encoded bytes has a minimum value of 61, so that it cannot be confused
// with the start character "#" (35) or the end character "\r" (13).
//
// Therefore, here is a summary of the total overhead:
//
//   - 1 extra byte per 3-bytes of data in a frame for encoding
//   - 4 extra bytes per data frame for identification and error checking
//   - 2 extra bits per byte to mark the start and end of a transmitted byte
//
// Here are some example maximum transfer rates for certain data sizes:
//
//   1-3 bytes -> 576 Hz
//   4-6 bytes -> 411 Hz
//   7-9 bytes -> 320 Hz
//   22-24 bytes -> 151 Hz
//   25-27 bytes -> 137 Hz
//   28-30 bytes -> 125 Hz
//   66 bytes -> 61.3 Hz (this is the size of Mikrokopter's DebugOut structure)

#include "serial_comms.h"

#include <util/crc16.h>

#include "adc.h"
#include "control.h"
#include "main.h"
#include "timing.h"
#include "uart.h"


// =============================================================================
// Private data:

#define TX_BUFFER_LENGTH (66)

static uint8_t tx_buffer_[TX_BUFFER_LENGTH];


// =============================================================================
// Private function declarations:

void MKSend(uint8_t address, uint8_t command, uint8_t * source,
  uint16_t length);


// =============================================================================
// Public functions:

void SendSensorData(void)
{
  struct SensorData {
    int16_t accelerometer_sum[3];
    int16_t gyro_sum[3];
    uint16_t biased_pressure;
    uint16_t battery_voltage;
    uint16_t timestamp;
  } sensor_data;

  sensor_data.accelerometer_sum[0] = AccelerometerSum()[0];
  sensor_data.accelerometer_sum[1] = AccelerometerSum()[1];
  sensor_data.accelerometer_sum[2] = AccelerometerSum()[2];
  sensor_data.gyro_sum[0] = GyroSum()[0];
  sensor_data.gyro_sum[1] = GyroSum()[1];
  sensor_data.gyro_sum[2] = GyroSum()[2];
  sensor_data.biased_pressure = BiasedPressure();
  sensor_data.battery_voltage = BatteryVoltage();
  sensor_data.timestamp = GetTimestamp();

  MKSend(1, 'I', (uint8_t *)&sensor_data, sizeof(sensor_data));
}

// -----------------------------------------------------------------------------
void SendKalmanData(void)
{
  struct KalmanData {
    int16_t gyro_sum[2];
    int16_t command[2];
    int16_t kalman_rate[2];
    int16_t kalman_acceleration[2];
    int16_t timestamp;
  } kalman_data;

  kalman_data.gyro_sum[0] = GyroSum()[0];
  kalman_data.gyro_sum[1] = GyroSum()[1];
  kalman_data.command[0] = (int16_t)(AttitudeCmd()[0] * 100.0);
  kalman_data.command[1] = (int16_t)(AttitudeCmd()[1] * 100.0);
  kalman_data.kalman_rate[0] = (int16_t)(KalmanP() * GYRO_SCALE
    * ADC_N_SAMPLES);
  kalman_data.kalman_rate[1] = (int16_t)(KalmanQ() * GYRO_SCALE
    * ADC_N_SAMPLES);
  kalman_data.kalman_acceleration[0] = (int16_t)(KalmanPDot() * GYRO_SCALE
    * ADC_N_SAMPLES * DT);
  kalman_data.kalman_acceleration[1] = (int16_t)(KalmanQDot() * GYRO_SCALE
    * ADC_N_SAMPLES * DT);
  kalman_data.timestamp = GetTimestamp();

  MKSend(1, 'I', (uint8_t *)&kalman_data, sizeof(kalman_data));
}


// =============================================================================
// Private functions:

void MKAddChecksum(uint16_t length)
{
  union checksum {
    uint16_t checksum;
    uint8_t bytes[2];
  };

  union checksum * checksum_ptr = (union checksum *)&tx_buffer_[length];

  checksum_ptr->checksum = 0;
  for (uint8_t i = length; i--; ) checksum_ptr->checksum += tx_buffer_[i];

  checksum_ptr->bytes[1] &= 0x0F;  // checksum % 4096
  uint8_t temp = checksum_ptr->bytes[0] & 0x3F;  // checksum % 64
  checksum_ptr->checksum >>= 6;  // checksum / 64
  checksum_ptr->bytes[0] += '=';
  checksum_ptr->bytes[1] = temp + '=';
}

// -----------------------------------------------------------------------------
void MKSend(uint8_t address, uint8_t command, uint8_t * source, uint16_t length)
{
  tx_buffer_[0] = '#';
  tx_buffer_[1] = 'a' + address;
  tx_buffer_[2] = command;

  uint8_t index = 3;
  while (length)
  {
    uint8_t i = 3, y[3] = { 0 };
    while (i-- && length--) y[i] = *(source++);

    tx_buffer_[index++] = '=' + (y[2] >> 2);
    tx_buffer_[index++] = '=' + (((y[2] & 0x03) << 4) | ((y[1] & 0xf0) >> 4));
    tx_buffer_[index++] = '=' + (((y[1] & 0x0f) << 2) | ((y[0] & 0xc0) >> 6));
    tx_buffer_[index++] = '=' + (y[0] & 0x3f);
  }

  MKAddChecksum(index);
  index += sizeof(uint16_t);

  tx_buffer_[index++] = '\r';

  UARTTxBytes(tx_buffer_, index);
}
