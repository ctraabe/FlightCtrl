#include "eeprom.h"


struct EEPROM EEMEM eeprom = {
  .n_motors = 8,
  .sbus_channel_pitch = 2,
  .sbus_channel_roll = 0,
  .sbus_channel_yaw = 3,
  .sbus_channel_thrust = 1,
  .sbus_channel_on_off = 17,
  .sbus_channel_switch = { 8, 9, 10, 11, 16, 13 },
  .sbus_channel_trim = { 3, 4, 5, 6 },
};
