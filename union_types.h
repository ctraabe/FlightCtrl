#ifndef UNION_TYPES_H_
#define UNION_TYPES_H_


#include <inttypes.h>


union S16Bytes {
  int16_t s16;
  uint8_t bytes[sizeof(int16_t)];
};

union S32Bytes {
  int32_t s32;
  uint8_t bytes[sizeof(int32_t)];
};

union U16Bytes {
  uint16_t u16;
  uint8_t bytes[sizeof(uint16_t)];
};

union U32Bytes {
  uint32_t u32;
  uint8_t bytes[sizeof(uint32_t)];
};


#endif  // UNION_TYPES_H_
