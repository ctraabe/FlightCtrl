#ifndef NAV_COMMS_H_
#define NAV_COMMS_H_


#include <inttypes.h>


enum NavStatusBits {
  NAV_STATUS_BIT_HEADING_DATA_OK         = 1<<0,
  NAV_STATUS_BIT_POSITION_DATA_OK        = 1<<1,
  NAV_STATUS_BIT_VELOCITY_DATA_OK        = 1<<2,
  NAV_STATUS_BIT_LOW_PRECISION_VERTICAL  = 1<<3,
  NAV_STATUS_BIT_POSITION_RESET_REQUEST  = 1<<4,
};

enum NavErrorBits {
  NAV_ERROR_BIT_STALE = 1<<0,
};

enum NavMode {
  NAV_MODE_OFF = 0,
  NAV_MODE_HOLD = 0x01,
  NAV_MODE_AUTO = 0x02,
  NAV_MODE_HOME = 0x03,
};


// =============================================================================
// Accessors:

uint8_t NavDataReady(void);

// -----------------------------------------------------------------------------
uint8_t NavRecieved(void);

// -----------------------------------------------------------------------------
enum NavMode NavMode(void);

// -----------------------------------------------------------------------------
uint8_t NavPositionReset(void);

// -----------------------------------------------------------------------------
uint8_t NavStatus(void);

// -----------------------------------------------------------------------------
uint8_t NavStatusOK(void);

// -----------------------------------------------------------------------------
const volatile float * PositionVector(void);

// -----------------------------------------------------------------------------
const volatile float * VelocityVector(void);

// -----------------------------------------------------------------------------
float HeadingCorrection0(void);

// -----------------------------------------------------------------------------
float HeadingCorrectionZ(void);

// -----------------------------------------------------------------------------
const volatile float * TargetPositionVector(void);

// -----------------------------------------------------------------------------
float TransitSpeed(void);

// -----------------------------------------------------------------------------
float TargetHeading(void);

// -----------------------------------------------------------------------------
float HeadingRate(void);

// -----------------------------------------------------------------------------
uint8_t NavStale(void);


// =============================================================================
// Public functions:

void NavCommsInit(void);

// -----------------------------------------------------------------------------
void ExchangeDataWithNav(void);

// -----------------------------------------------------------------------------
void NotifyNav(void);

// -----------------------------------------------------------------------------
void ProcessDataFromNav(void);

// -----------------------------------------------------------------------------
void RequestNavRoute(uint8_t nav_route);

// -----------------------------------------------------------------------------
void ResetPositionHold(void);


#endif  // NAV_COMMS_H_
