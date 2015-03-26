#include "eeprom.h"

#include "adc.h"


struct EEPROM EEMEM eeprom = {
  .n_motors = 8,
};
