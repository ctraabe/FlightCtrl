#ifndef ATTITUDE_H_
#define ATTITUDE_H_


#include <inttypes.h>


// =============================================================================
// Accessors:

const float * GravityInBodyVector(void);

// -----------------------------------------------------------------------------
float HeadingAngle(void);

// -----------------------------------------------------------------------------
const float * Quat(void);


// =============================================================================
// Public functions:

void ResetAttitude(void);

// -----------------------------------------------------------------------------
void CorrectHeading(void);

// -----------------------------------------------------------------------------
void UpdateAttitude(void);

// -----------------------------------------------------------------------------
float * UpdateGravtiyInBody(const float quat[4], float g_b[3]);

// -----------------------------------------------------------------------------
float * UpdateQuaternion(float quat[4], const float angular_rate[3], float dt);

// -----------------------------------------------------------------------------
void EulerAnglesFromQuaternion(const float quat[4], float * phi, float * theta,
  float * psi);

// -----------------------------------------------------------------------------
float HeadingFromQuaternion(const float quat[4]);


#endif  // ATTITUDE_H_
