#ifndef GLOBAL_DEFINITIONS_H_
#define GLOBAL_DEFINITIONS_H_


#define FS (128.0)
#define DT (1.0 / FS)

#define MAX_MOTORS (8)

enum BodyAxes {
  X_BODY_AXIS = 0,
  Y_BODY_AXIS = 1,
  Z_BODY_AXIS = 2,
};

// =============================================================================
// Public functions:

void PreflightInit(void);

// -----------------------------------------------------------------------------
void SensorCalibration(void);


#endif  // GLOBAL_DEFINITIONS_H_
