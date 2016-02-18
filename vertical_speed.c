#include "vertical_speed.h"

#include "adc.h"
#include "attitude.h"
#include "main.h"
#include "pressure_altitude.h"
#include "vector.h"


// =============================================================================
// Private data:

#define COMPLIMENTARY_GAIN (0.005)

static float vertical_acceration_ = 0.0, vertical_speed_ = 0.0;


// =============================================================================
// Accessors:

float VerticalAcceleration(void)
{
  return vertical_acceration_;
}

// -----------------------------------------------------------------------------
float VerticalSpeed(void)
{
  return vertical_speed_;
}


// =============================================================================
// Public functions:

void UpdateVerticalSpeed(void)
{
  static float pressure_altitude_pv = 0.0;

  // Compute the projection of the acceleration reported by the accelerometer
  // along the inertial vertical axis.
  vertical_acceration_ = (1.0 + Vector3Dot(GravityInBodyVector(),
    AccelerationVector())) * -GRAVITY_ACCELERATION;  // positive up

  // Compute the derivative of the pressure altitude.
  float d_pressure_altitude = (DeltaPressureAltitude() - pressure_altitude_pv)
    / DT;
  pressure_altitude_pv = DeltaPressureAltitude();

  // Complimentary (Kalman) filter the integral of the vertical acceleration
  // reported by the accelerometer and the derivative of the pressure altitude.
  vertical_speed_ = (1.0 - COMPLIMENTARY_GAIN) * (vertical_speed_
    + vertical_acceration_ * DT) + COMPLIMENTARY_GAIN * d_pressure_altitude;
}
