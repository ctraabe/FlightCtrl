#include "ut_serial_tx.h"

#include "adc.h"
#include "nav_comms.h"
#include "pressure_altitude.h"
#include "timing.h"
#include "ut_serial_protocol.h"
#include "vertical_speed.h"


// =============================================================================
// Public functions:

void SendVerticalData(void)
{
  struct VerticalData {
    uint16_t timestamp;
    uint16_t biased_pressure;
    float vertical_acceleration;
    float vertical_speed;
    float pressure_altitude;
    float vertical_speed_from_nav;
    float altitude_from_nav;
  } __attribute__((packed)) vertical_data;

  _Static_assert(UT_HEADER_LENGTH + sizeof(struct VerticalData) + 2
    < UART_TX_BUFFER_LENGTH,
    "VerticalData is too large for the UART TX buffer");

  vertical_data.timestamp = GetTimestamp();
  vertical_data.biased_pressure = BiasedPressureSum();
  vertical_data.vertical_acceleration = VerticalAcceleration();
  vertical_data.vertical_speed = VerticalSpeed();
  vertical_data.pressure_altitude = DeltaPressureAltitude();
  vertical_data.vertical_speed_from_nav = -VelocityVector()[D_WORLD_AXIS];
  vertical_data.altitude_from_nav = -PositionVector()[D_WORLD_AXIS];

  UTSerialTx(1, (uint8_t *)&vertical_data, sizeof(vertical_data));
}
