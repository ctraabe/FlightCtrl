// This file provides a structure and initial conditions for placing persistent
// variables in EEPROM. This method ensures that variables remain in the same
// memory location every for every compilation. This means that EEPROM doesn't
// have to be cleared when the chip is reprogrammed, and previously stored
// values will remain even after programming.

#ifndef EEPROM_H_
#define EEPROM_H_


#include <inttypes.h>
#include <avr/eeprom.h>

#include "main.h"


extern struct EEPROM {
  uint8_t space[2048];  // 2kB space reserved for original MikroKopter firmware
  float actuation_inverse[MAX_MOTORS][4];  // Inverse of the actuation matrix
  int16_t acc_offset[3];
  int16_t gyro_offset[3];
  uint8_t n_motors;
  uint8_t pressure_coarse_bias;
  uint16_t coarse_bias_steps_to_pressure_steps;
  uint16_t fine_bias_steps_to_pressure_steps;
  uint8_t sbus_channel_pitch;
  uint8_t sbus_channel_roll;
  uint8_t sbus_channel_yaw;
  uint8_t sbus_channel_thrust;
  uint8_t sbus_channel_on_off;
  uint8_t sbus_channel_horizontal_control;
  uint8_t sbus_channel_vertical_control;
  uint8_t sbus_channel_switch[6];
  uint8_t sbus_channel_trim[4];
} eeprom;


#endif  // EEPROM_H_
