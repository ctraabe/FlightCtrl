; This interrupt handler increments a 16-bit word (ms_timestamp_) every time
; TIMER1 reaches the value in ICR1 (which should occur at 1kHz).

; The following references were very helpful in making this file:
; 8-bit AVR Instruction Set
; Atmel AT1886: Mixing Assembly and C with AVRGCC (has many errors though)
; ATmega164A/PA/324A/PA/644A/PA/1284/P Datasheet (Instruction Set Summary)

; Stack usage: 2 bytes
; Typical runtime: 20 cycles
; Worst case runtime: 29 cycles

; Encapsulating the include in a .nolist statement prevents a bunch of
; unnecessary output in the .lst file.
.nolist
#include <avr/io.h>
.list

.extern ms_timestamp_

__SREG__ = _SFR_IO_ADDR(SREG)

.section .text.TIMER1_CAPT_vect,"ax",@progbits
.global TIMER1_CAPT_vect
TIMER1_CAPT_vect:
  push r0  ; Save r0 to the stack
  push r1  ; Save r1 to the stack
  in r1, __SREG__  ; Save SREG in r1
  lds r0, ms_timestamp_  ; Load the lower byte from &ms_timestamp_
  inc r0  ; Increment the lower byte
  breq MS_HI  ; If r0 rolled over to 0, then branch to MS_HI

MS_SAVE:
  sts ms_timestamp_, r0  ; Save the lower byte to &ms_timestamp_
  out __SREG__, r1  ; Restore the state of SREG
  pop r1  ; Restore r1 from the stack
  pop r0  ; Restore r0 from the stack
  reti

MS_HI:
  lds r0, ms_timestamp_ + 1  ; Load the upper byte from &ms_timestamp_
  inc r0  ; Increment the upper byte of ms_timestamp_
  sts ms_timestamp_ + 1, r0  ; Save the upper byte to &ms_timestamp_
  clr r0  ; Restore r0 to 0
  rjmp MS_SAVE  ; Jump back to save lower byte
