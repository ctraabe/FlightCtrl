#ifndef MK_SERIAL_TX_H_
#define MK_SERIAL_TX_H_


#include <inttypes.h>


enum MKTxBits {
  MK_TX_BLCONFIG                = 1<<0,
  MK_TX_BLCONFIG_SAVE_CONFIRM   = 1<<1,
  MK_TX_MIXER                   = 1<<2,
  MK_TX_MIXER_SAVE_CONFIRM      = 1<<3,
  MK_TX_MK_DATA                 = 1<<4,
  MK_TX_OUTPUT_LABEL            = 1<<5,
  MK_TX_PTAM_DATA               = 1<<6,
  MK_TX_RC                      = 1<<7,
  MK_TX_SETTINGS_READ           = 1<<8,
  MK_TX_SETTINGS_SAVE_CONFIRM   = 1<<9,
  MK_TX_SETTINGS_SWITCH_CONFIRM = 1<<10,
  MK_TX_VERSION                 = 1<<11,
};

enum MKStream {
  MK_STREAM_NONE = 0,
  MK_STREAM_CONTROL,
  MK_STREAM_DEBUG,
  MK_STREAM_KALMAN,
  MK_STREAM_MOTOR_SETPOINTS,
  MK_STREAM_SENSORS,
};


// =============================================================================
// Public functions:

// This function sends data that has been requested.
void SendPendingMKSerial(void);

// -----------------------------------------------------------------------------
// This function starts the specified data stream at the specified period. Note
// that this stream has to be renewed periodically by resending the request. If
// no renewing request is received, then the stream will time out after a while.
// Also note that the stream output period will be quantized to the main control
// frequency.
void SetMKDataStream(enum MKStream mk_stream, uint16_t period_10ms);

// -----------------------------------------------------------------------------
// This function sets a one-time request for data.
void SetMKTxRequest(enum MKTxBits);


#endif  // MK_SERIAL_TX_H_
