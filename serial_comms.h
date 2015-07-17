#ifndef SERIAL_COMMS_H_
#define SERIAL_COMMS_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

void SendSensorData(void);

// -----------------------------------------------------------------------------
void SendKalmanData(void);

// -----------------------------------------------------------------------------
void SendMotorSetpoints(void);


#endif  // SERIAL_COMMS_H_
