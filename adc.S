; This file provides the interrupt handler for the ADC. It first records the
; previous sample, then sets the next channel (sensor) to be read, and initiates
; the analog-to-digital conversion on that channel. Readings are recorded into a
; ring array (such that the oldest sample in the array is replaced with the
; newest). This interrupt is triggered at 20,000,000 / 128 / 13 ~ 12kHz.

; This interrupt handler performs the following equivalent C code, but has been
; hand optimized to reduce the number of cycles required by nearly 50% (freeing
; about 2% total processing):
;   samples_index_ = (samples_index_ + 1) % (ADC_N_SAMPLES * ADC_N_CHANNELS);
;   ADMUX = samples_index_ % ADC_N_CHANNELS;
;   samples_[samples_index_] = ADC;

; Stack usage: 5 bytes
; Runtime: 48 cycles

; The following references were very helpful in making this file:
; 8-bit AVR Instruction Set
; Atmel AT1886: Mixing Assembly and C with AVRGCC (has many errors though)
; ATmega164A/PA/324A/PA/644A/PA/1284/P Datasheet (Instruction Set Summary)

; Encapsulating the include in a .nolist statement prevents a bunch of
; unnecessary output in the .lst file.
.nolist
#include "adc.h"
#include <avr/io.h>
.list

.extern samples_  ; uint16_t[ADC_N_SAMPLES][ADC_N_CHANNELS]
.extern samples_index_  ; uint8_t

__SREG__ = _SFR_IO_ADDR(SREG)


.section .text.ADC_vect,"ax",@progbits
.global ADC_vect
ADC_vect:
  ; Save the state of SREG to be restored before returning.
  push r0
  in r0, __SREG__  ; Save SREG in R0

  ; Free up some registers by pushing their contents to the stack.
  push XL  ; Save XL (R26) to the stack
  push XH  ; Save XH (R27) to the stack
  push YL  ; Save YL (R28) to the stack
  push YH  ; Save YH (R29) to the stack

  ; samples_index_ = (samples_index_ + 1) % (ADC_N_SAMPLES * ADC_N_CHANNELS)
  lds YL, samples_index_  ; Load the value at SRAM &samples_index_ into YL
  inc YL  ; YL++
  andi YL, (ADC_N_SAMPLES * ADC_N_CHANNELS - 1)  ; YL % ...
  sts samples_index_, YL  ; Save the value in YL to &samples_index_

  ; ADMUX = samples_index_ % ADC_N_CHANNELS
  mov YH, YL  ; Copy YL to YH
  andi YH, (ADC_N_CHANNELS - 1)  ; YH % 8
  sts ADMUX, YH  ; Set ADMUX to the value in YH

  ; samples_[samples_index_] = ADC;
  lds XL, ADCL  ; Load the lower ADC byte in into XL
  lds XH, ADCH  ; Load the upper ADC byte in into XH
  clr YH
  add YL, YL  ; Double the index since samples_ is an array of 2-byte values
  subi YL, lo8(-(samples_))  ; Add &samples[0] to 2*samples_index_ (lower byte)
  sbci YH, hi8(-(samples_))  ; Add &samples[0] to 2*samples_index_ (upper byte)
  std Y+1, XH  ; Put the byte in XH to the SRAM address in Y + 1
  st Y, XL  ; Put the byte in XL to the SRAM address in Y

  ; Restore the state of SREG
  out __SREG__, r0

  ; Restore the state of the freed registers (in order).
  pop YH  ; Restore YH (R28) from the stack
  pop YL  ; Restore YL (R28) from the stack
  pop XH  ; Restore XH (R27) from the stack
  pop XL  ; Restore XL (R26) from the stack
  pop r0
  reti
