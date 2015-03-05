#ifndef SBUS_H_
#define SBUS_H_


#define SBUS_MESSAGE_LENGTH (25 - 1)  // Start byte is not recorded
#define SBUS_RX_BUFFER_LENGTH (SBUS_MESSAGE_LENGTH + 2)  // Includes timestamp
#define SBUS_START_BYTE (0x0F)
#define SBUS_END_BYTE (0x00)


#ifndef __ASSEMBLER__


// =============================================================================
// Public functions:

void SBusInit(void);


#endif // __ASSEMBLER__

#endif // SBUS_H_
