#include "altitude.h"

#include "adc.h"
#include "attitude.h"
#include "main.h"
#include "vector.h"


// =============================================================================
// Private data:

static float altitude_ = 0.0, vertical_speed_ = 0.0;


// =============================================================================
// Accessors:

float Altitude(void)
{
  return altitude_;
}

// -----------------------------------------------------------------------------
float VerticalSpeed(void)
{
  return vertical_speed_;
}


// =============================================================================
// Public functions:

void ResetAltitude(void)
{
  altitude_ = 0;
}

// -----------------------------------------------------------------------------
void UpdateAltitude(void)
{
  float vertical_acceration = 1.0 - VectorDot(GravityInBodyVector(),
    AccelerationVector());  // g
  vertical_speed_ += vertical_acceration * DT;
  altitude_ += vertical_speed_ * DT;
}
