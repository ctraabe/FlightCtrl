#include "eeprom.h"


struct EEPROM EEMEM eeprom = {
  .n_motors = 8,
  .sbus_channel_pitch = 2,
  .sbus_channel_roll = 3,
  .sbus_channel_yaw = 0,
  .sbus_channel_thrust = 1,
  .sbus_channel_on_off = 17,
  .sbus_channel_altitude_control = 16,
  .sbus_channel_nav_control = 5,
  .sbus_channel_takeoff = 7,
  .sbus_channel_go_home = 6,
  .sbus_channel_switch = { 4, 12, 13, 14, 15 },
  .sbus_channel_trim = { 8, 9, 10, 11 },
};
