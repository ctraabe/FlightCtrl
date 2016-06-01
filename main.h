#ifndef MAIN_H_
#define MAIN_H_


#define FS (128.0)
#define DT (1.0 / FS)
#define GRAVITY_ACCELERATION (9.8)

#define MAX_MOTORS (8)

enum BodyAxes {
  X_BODY_AXIS = 0,
  Y_BODY_AXIS = 1,
  Z_BODY_AXIS = 2,
};

enum WorldAxes {
  N_WORLD_AXIS = 0,  // North
  E_WORLD_AXIS = 1,  // East
  D_WORLD_AXIS = 2,  // Down
};


// =============================================================================
// Accessors:

uint8_t BoardVersion(void);


// =============================================================================
// Public functions:

void PreflightInit(void);

// -----------------------------------------------------------------------------
void SensorCalibration(void);


#endif  // MAIN_H_
