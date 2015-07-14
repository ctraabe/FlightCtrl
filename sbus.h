#ifndef SBUS_H_
#define SBUS_H_


// =============================================================================
// Definitions:

#define SBUS_MESSAGE_LENGTH (25 - 1)  // Start byte is not recorded
#define SBUS_RX_BUFFER_LENGTH (SBUS_MESSAGE_LENGTH + 2)  // Includes timestamp
#define SBUS_START_BYTE (0x0F)
#define SBUS_END_BYTE (0x00)
#define SBUS_MAX (672)


#ifndef __ASSEMBLER__


#include <inttypes.h>


enum SBusErrorBits {
  SBUS_ERROR_BIT_STALE = 1<<0,
};


// =============================================================================
// Accessors:

uint8_t SBusErrorBits(void);

// -----------------------------------------------------------------------------
int16_t SBusPitch(void);

// -----------------------------------------------------------------------------
int16_t SBusRoll(void);

// -----------------------------------------------------------------------------
int16_t SBusYaw(void);

// -----------------------------------------------------------------------------
int16_t SBusThrust(void);

// -----------------------------------------------------------------------------
uint8_t SBusOnOff(void);

// -----------------------------------------------------------------------------
uint8_t SBusStale(void);


// =============================================================================
// Public functions:

void SBusInit(void);

// -----------------------------------------------------------------------------
void SBusSetChannels(uint8_t pitch, uint8_t roll, uint8_t yaw, uint8_t thrust,
  uint8_t on_off);

// -----------------------------------------------------------------------------
uint8_t SBusThrustStickDown(void);

// -----------------------------------------------------------------------------
uint8_t SBusThrustStickUp(void);

// -----------------------------------------------------------------------------
uint8_t SBusYawStickLeft(void);

// -----------------------------------------------------------------------------
uint8_t SBusYawStickRight(void);

// -----------------------------------------------------------------------------
void UpdateSBus(void);


#endif  // __ASSEMBLER__

#endif  // SBUS_H_
