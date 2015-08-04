#include "mk_serial_tx.h"

#include "adc.h"
#include "attitude.h"
#include "control.h"
#include "mk_serial_protocol.h"
#include "motors.h"
#include "sbus.h"
#include "state.h"
#include "timing.h"
#include "union_types.h"


// =============================================================================
// Private data:

#define STREAM_TIMEOUT (4000)  // ms

static uint16_t tx_request_ = 0x0000;
static enum MKStream mk_stream_ = MK_STREAM_NONE;
static uint16_t stream_period_ = 0, stream_timer_ = 0, stream_timeout_ = 0;


// =============================================================================
// Private function declarations:

static void SendControlData(void);
static void SendKalmanData(void);
static void SendMotorSetpoints(void);
static void SendSensorData(void);
static void SendVersion(void);


// =============================================================================
// Public functions:

// This function sends data that has been requested.
void SendPendingMKSerial(void)
{
  // Handle only one request at a time.
  if (tx_request_)
  {
    // A one-time request has higher priority than a periodic "stream" of data.
    if (tx_request_ & MK_TX_VERSION) SendVersion();
  }
  else if (mk_stream_ && TimestampInPast(stream_timer_))
  {
    // A data stream is active and it is time for another transmission.
    switch (mk_stream_)
    {
      case MK_STREAM_CONTROL:
        SendControlData();
        break;
      case MK_STREAM_KALMAN:
        SendKalmanData();
        break;
      case MK_STREAM_MOTOR_SETPOINTS:
        SendMotorSetpoints();
        break;
      case MK_STREAM_SENSORS:
        SendSensorData();
        break;
      default:
        break;
    }
    stream_timer_ += stream_period_;

    // Prevent timer rollover for small periods.
    if (TimestampInPast(stream_timer_)) stream_timer_ = GetTimestamp();

    // Disable the stream if no request has been renewing received in a while.
    if (TimestampInPast(stream_timeout_)) mk_stream_ = MK_STREAM_NONE;
  }
}

// -----------------------------------------------------------------------------
// This function starts the specified data stream at the specified period. Note
// that this stream has to be renewed periodically by resending the request. If
// no renewing request is received, then the stream will time out after a while.
// Also note that the stream output period will be quantized to the main control
// frequency.
void SetMKDataStream(enum MKStream mk_stream, uint16_t period_10ms)
{
  mk_stream_ = mk_stream;

  uint16_t stream_period = period_10ms * 10;  // ms
  if (!stream_period_)
    stream_timer_ = GetTimestampMillisFromNow(0);  // Start stream immediately
  else if (stream_period < stream_period_)
    stream_timer_ = GetTimestampMillisFromNow(stream_period);
  stream_period_ = stream_period;
  stream_timeout_ = GetTimestampMillisFromNow(STREAM_TIMEOUT);
}

// -----------------------------------------------------------------------------
// This function sets a one-time request for data.
void SetMKTxRequest(enum MKTxBits tx_request)
{
  tx_request_ |= tx_request;
}


// =============================================================================
// Private functions:

static void SendControlData(void)
{
  struct DebugData {
    int16_t accelerometer_sum[3];
    int16_t gyro_sum[3];
    int16_t stick_16[2];
    uint8_t stick_8[2];
  } __attribute__((packed)) debug_data;

  union S16Bytes temp;

  debug_data.accelerometer_sum[0] = AccelerometerSum(X_BODY_AXIS);
  debug_data.accelerometer_sum[1] = AccelerometerSum(Y_BODY_AXIS);
  debug_data.accelerometer_sum[2] = AccelerometerSum(Z_BODY_AXIS);
  debug_data.gyro_sum[0] = GyroSum(X_BODY_AXIS);
  debug_data.gyro_sum[1] = GyroSum(Y_BODY_AXIS);
  debug_data.gyro_sum[2] = GyroSum(Z_BODY_AXIS);
  temp.s16 = SBusYaw();
  debug_data.stick_16[0] = (SBusPitch() << 4) | GetDebugResetAttitude()
    | (temp.bytes[1] & 0x07);
  debug_data.stick_8[0] = temp.bytes[0];
  temp.s16 = SBusThrust();
  debug_data.stick_16[1] = (SBusRoll() << 4) | MotorsRunning()
    | (temp.bytes[1] & 0x07);
  debug_data.stick_8[1] = temp.bytes[0];

  MKSerialTx(1, 'I', (uint8_t *)&debug_data, sizeof(debug_data));
}

// -----------------------------------------------------------------------------
static void SendKalmanData(void)
{
  struct KalmanData {
    int16_t gyro_sum[2];
    int16_t command[2];
    int16_t kalman_rate[2];
    int16_t kalman_acceleration[2];
    uint16_t timestamp;
  } __attribute__((packed)) kalman_data;

  kalman_data.gyro_sum[0] = GyroSum(X_BODY_AXIS);
  kalman_data.gyro_sum[1] = GyroSum(Y_BODY_AXIS);
  kalman_data.command[0] = (int16_t)(AngularCommand(X_BODY_AXIS) * 100.0);
  kalman_data.command[1] = (int16_t)(AngularCommand(Y_BODY_AXIS) * 100.0);
  kalman_data.kalman_rate[0] = (int16_t)(KalmanP() * GYRO_SCALE
    * ADC_N_SAMPLES);
  kalman_data.kalman_rate[1] = (int16_t)(KalmanQ() * GYRO_SCALE
    * ADC_N_SAMPLES);
  kalman_data.kalman_acceleration[0] = (int16_t)(KalmanPDot() * GYRO_SCALE
    * ADC_N_SAMPLES * DT);
  kalman_data.kalman_acceleration[1] = (int16_t)(KalmanQDot() * GYRO_SCALE
    * ADC_N_SAMPLES * DT);
  kalman_data.timestamp = GetTimestamp();

  MKSerialTx(1, 'I', (uint8_t *)&kalman_data, sizeof(kalman_data));
}

// -----------------------------------------------------------------------------
static void SendMotorSetpoints(void)
{
  struct MotorSetpoints {
    int16_t motor_setpoints[MAX_MOTORS];
    uint16_t timestamp;
  } __attribute__((packed)) motor_setpoints;

  for (uint16_t i = MAX_MOTORS; i--; )
    motor_setpoints.motor_setpoints[i] = MotorSetpoint(i);
  motor_setpoints.timestamp = GetTimestamp();

  MKSerialTx(1, 'I', (uint8_t *)&motor_setpoints, sizeof(motor_setpoints));
}

// -----------------------------------------------------------------------------
static void SendSensorData(void)
{
  struct SensorData {
    int16_t accelerometer_sum[3];
    int16_t gyro_sum[3];
    uint16_t biased_pressure;
    uint16_t battery_voltage;
    uint16_t timestamp;
  } __attribute__((packed)) sensor_data;

  sensor_data.accelerometer_sum[0] = AccelerometerSum(X_BODY_AXIS);
  sensor_data.accelerometer_sum[1] = AccelerometerSum(Y_BODY_AXIS);
  sensor_data.accelerometer_sum[2] = AccelerometerSum(Z_BODY_AXIS);
  sensor_data.gyro_sum[0] = GyroSum(X_BODY_AXIS);
  sensor_data.gyro_sum[1] = GyroSum(Y_BODY_AXIS);
  sensor_data.gyro_sum[2] = GyroSum(Z_BODY_AXIS);
  sensor_data.biased_pressure = BiasedPressure();
  sensor_data.battery_voltage = BatteryVoltage();
  sensor_data.timestamp = GetTimestamp();

  MKSerialTx(1, 'I', (uint8_t *)&sensor_data, sizeof(sensor_data));
}

// -----------------------------------------------------------------------------
static void SendVersion(void)
{
  MKSerialTx(1, 'V', 0, 0);
  tx_request_ &= ~MK_TX_VERSION;
}
