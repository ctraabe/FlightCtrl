; This file provides some common timing functions such as wait, check if some
; time has elapsed, and measure how much time has elapsed. This file assumes
; that TIMER3 has been initialized to produce a an overflow interrupt at 128 Hz
; by setting the clock divider to 8 and TOP to 19531.

; The following references were very helpful in making this file:
; 8-bit AVR Instruction Set
; Atmel AT1886: Mixing Assembly and C with AVRGCC (has many errors though)
; ATmega164A/PA/324A/PA/644A/PA/1284/P Datasheet (Instruction Set Summary)

; Worst case stack usage: 2 bytes
; Worst case runtime: 31 cycles

; Encapsulating the include in a .nolist statement prevents a bunch of
; unnecessary output in the .lst file.
.nolist
#include <avr/io.h>
.list

.extern ms_timestamp_

__SREG__ = _SFR_IO_ADDR(SREG)


; This interrupt handler increments a 16-bit word every time TIMER1 reaches the
; value in ICR1 (which should occur at 1kHz).
;;
.section .text.TIMER1_CAPT_vect,"ax",@progbits
.global TIMER1_CAPT_vect
TIMER1_CAPT_vect:
  in r1, __SREG__  ; Save SREG in R1 (normally reserved for 0)
  push r26  ; Save R26 (X-) to the stack
  lds r26, ms_timestamp_  ; Load the lower byte of ms_timestamp
  inc r26  ; Increment the lower byte
  breq MS_HI  ; If r26 rolled over to 0, then branch to MS_HI

MS_SAVE:
  sts ms_timestamp_, r26  ; Save lower byte to ms_timestamp
  mov r26, r1  ; (because clr will set the Z flag)
  clr r1  ; Restore R1 to 0
  out __SREG__, r26  ; Restore the state of SREG
  pop r26  ; Restore R26 (X-) from the stack
  reti

MS_HI:
  push r27  ; Save R27 (X+) to the stack
  lds r27, ms_timestamp_ + 1  ; Load the upper byte of ms_timestamp
  inc r27  ; Increment the upper byte of ms_timestamp
  sts ms_timestamp_ + 1, r27  ; Save the upper byte to ms_timestamp
  pop r27  ; Restore R27 (X+) from the stack
  rjmp MS_SAVE  ; Jump back to save lower byte
