#ifndef _TIMING_H
#define _TIMING_H

#define TIMER3_DIVIDER 8
#define F_ICR3 128
#define F_OCR3A 1000

#ifndef __ASSEMBLER__

  #include <inttypes.h>

  void TimingInit(void);

  uint16_t GetTimestampMillisFromNow(uint16_t t);
  uint8_t TimestampInPast(uint16_t t);
  uint16_t MillisSinceTimestamp(uint16_t *last_time);

  void Wait(uint16_t);

#endif // __ASSEMBLER__

#endif // TIMING_H_
