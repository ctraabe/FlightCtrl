#include "eeprom.h"


struct EEPROM EEMEM eeprom = {
  .n_motors = 8,
  .sbus_channel_pitch = 2,
  .sbus_channel_roll = 3,
  .sbus_channel_yaw = 0,
  .sbus_channel_thrust = 1,
  .sbus_channel_on_off = 17,
  .sbus_channel_horizontal_control = 5,
  .sbus_channel_vertical_control = 4,
  .sbus_channel_switch = { 6, 7, 4, 5, 16, 13 },
  .sbus_channel_trim = { 8, 9, 10, 11 },
};
