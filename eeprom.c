#include "eeprom.h"

#include "adc.h"


struct EEPROM EEMEM eeprom = {
  .acc_offset = { 0 },
};
