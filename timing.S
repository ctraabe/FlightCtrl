; This file provides some common timing functions such as wait, check if some
; time has elapsed, and measure how much time has elapsed. This file assumes
; that TIMER3 has been initialized to produce a an overflow interrupt at 128 Hz
; by setting the clock divider to 8 and TOP to 19531.

; The following references were very helpful in making this file:
; 8-bit AVR Instruction Set
; Atmel AT1886: Mixing Assembly and C with AVRGCC (has many errors though)
; ATmega164A/PA/324A/PA/644A/PA/1284/P Datasheet (Instruction Set Summary)

; Encapsulating the include in a .nolist statement prevents a bunch of
; unnecessary output in the .lst file.
.nolist
#include <avr/io.h>
#include "timing.h"
.list

.extern ms_timestamp_

__SREG__ = _SFR_IO_ADDR(SREG)

; This interrupt handler is activated every time TIMER3 reaches the value in
; OCR3A. It first increments the 16-bit value in the static variable
; ms_timestamp_, and then updates OCR3A so that this routine will be called
; every millisecond. The formula is OCR3A += 2500, where 2500 = F_CPU /
; TIMER3_DIVIDER / 1000. If OCR3A > ICR3 (TOP of TIMER3), then OCR3A -= (ICR3 +
; 1), where ICR3 = 19531.
;;
.global TIMER3_COMPA_vect
TIMER3_COMPA_vect:
  in r1, __SREG__  ; Save SREG in R1 (normally reserved for 0)
  push r26  ; Save R26 (X-) to the stack
  push r27  ; Save R27 (X+) to the stack
  lds r26, ms_timestamp_  ; Load the lower byte of ms_timestamp
  inc r26  ; Increment the lower byte
  breq MS_HI  ; If r26 rolled over to 0, then branch to MS_HI
MS_OCR3A:
  sts ms_timestamp_, r26  ; Save lower byte to ms_timestamp
  lds r26, OCR3AL  ; Load the lower byte of OCR3A
  lds r27, OCR3AH  ; Load the lower byte of OCR3A
  subi r26, (-F_CPU_S / TIMER3_DIVIDER / F_OCR3A) & 0xFF
  sbci r27, ((-F_CPU_S / TIMER3_DIVIDER / F_OCR3A) & 0xFF00) >> 8
  sts OCR3AH, r27  ; Save upper byte of OCR3A
  sts OCR3AL, r26  ; Save lower byte of OCR3A
  subi r26, (F_CPU_S / TIMER3_DIVIDER / F_ICR3) & 0xFF
  sbci r27, (F_CPU_S / TIMER3_DIVIDER / F_ICR3) >> 8
  brlo MS_RET  ; If result was negative, then branch to MS_RET
  sts OCR3AH, r27  ; Save upper byte of OCR3A
  sts OCR3AL, r26  ; Save lower byte of OCR3A
MS_RET:
  mov r26, r1  ; (because clr will set the Z flag)
  clr r1  ; Restore R1 to 0
  out __SREG__, r26  ; Restore the state of SREG
  pop r27  ; Restore R27 (X+) from the stack
  pop r26  ; Restore R26 (X-) from the stack
  reti;
MS_HI:
  lds r27, ms_timestamp_ + 1  ; Load the upper byte of ms_timestamp
  inc r27  ; Increment the upper byte of ms_timestamp
  sts ms_timestamp_ + 1, r27  ; Save the upper byte to ms_timestamp
  rjmp MS_OCR3A  ; Jump back to save lower byte

; uint16_t GetTimestampMillisFromNow(uint16_t t);
;;
; This function returns a timestamp corresponding to "t" ms in the future. This
; timestamp can be checked against the current timestamp to see if a certain
; amount of time has passed. This function works for delays up to 32767 ms.
;;
; Note that the TIMER3_COMPA interrupt might occur between the 2 load (lds)
; instructions. This could update the high part of ms_timestamp_ after it has
; been loaded in r26, giving unexpected results. Interrupts are disabled and re-
; enabled to avoid this.
;;
.global GetTimestampMillisFromNow
GetTimestampMillisFromNow:
  cli
  lds r26, ms_timestamp_
  lds r27, ms_timestamp_ + 1
  sei
  adiw r26, 1  ; Add 1 in case we are close to the next tick.
  add r24, r26
  adc r25, r27
  ret

; int8_t TimestampInPast(uint16_t t);
;;
; This function compares a timestamp to the current timestamp and returns a 1 if
; the timestamp is in the past. This function works for delays up to 32767 ms.
;;
.global TimestampInPast
TimestampInPast:
  cli
  lds r26, ms_timestamp_
  lds r27, ms_timestamp_ + 1
  sei
  sub r24, r26
  sbc r25, r27
  ldi r24, 0x01  ; Set the return value to 1
  brmi .+2  ; If the result is negative, then skip an instruction.
  ldi r24, 0x00  ; Set the return value to 0
  ret

; uint16_t MillisSinceTimestamp(uint16_t *last_time);
;;
; This function returns the amount of time that has elapsed since the timestamp
; "last_time" has occurred. This function works for time periods up to 32767 ms.
; The function also automatically updates last_time so that it can be easily be
; called periodically.
;;
.global MillisSinceTimestamp
MillisSinceTimestamp:
  movw r30, r24  ; Copy address of "last_time" to the Z register pair (R31:30)
  cli
  lds r24, ms_timestamp_
  lds r25, ms_timestamp_ + 1
  sei
  ld r26, Z  ; Load the byte at Z (lower bytes of "last_time") to R24
  ldd r27, Z + 1  ; Load the byte at Z+1 (upper bytes of "last_time") to R25
  st Z, r24  ; Store the current timestamp to "last_time"
  std Z + 1, r25
  sub r24,r26  ; Subtract "last_time" from the current timestamp
  sbc r25,r27
  ret
