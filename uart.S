; This file provides the RX interrupt handler for the USART0. It saves incoming
; bytes to a ring buffer.

; This interrupt handler performs the following equivalent C code, but has been
; hand optimized to reduce the number of cycles required:
;   rx_buffer_head = (rx_buffer_head + 1) % RX_BUFFER_LENGTH;
;   rx_buffer_[rx_buffer_head] = UDR0;

; Stack usage: 4 bytes
; Runtime: 33 cycles

; The following references were very helpful in making this file:
; 8-bit AVR Instruction Set
; Atmel AT1886: Mixing Assembly and C with AVRGCC (has many errors though)
; ATmega164A/PA/324A/PA/644A/PA/1284/P Datasheet (Instruction Set Summary)

; Encapsulating the include in a .nolist statement prevents a bunch of
; unnecessary output in the .lst file.
.nolist
#include "uart.h"
#include <avr/io.h>
.list

.extern rx_buffer_  ; uint8_t[RX_BUFFER_LENGTH]
.extern samples_index_  ; uint8_t

__SREG__ = _SFR_IO_ADDR(SREG)


; This interrupt handler increments a 16-bit word every time TIMER1 reaches the
; value in ICR1 (which should occur at 1kHz).
;;
.section .text.USART0_RX_vect,"ax",@progbits
.global USART0_RX_vect
USART0_RX_vect:
  ; Save the state of SREG to be restored before returning.
  push r0
  in r0, __SREG__  ; Save SREG in R0

  ; Free up some registers by pushing their contents to the stack.
  push r1  ; Save r1 to the stack
  push YL  ; Save YL (R28) to the stack
  push YH  ; Save YH (R29) to the stack

  ; rx_buffer_head_ = (rx_buffer_head_ + 1) % RX_BUFFER_LENGTH
  lds YL, rx_buffer_head_  ; Load the value at SRAM &rx_buffer_head_ into YL
  inc YL  ; YL++
  andi YL, (RX_BUFFER_LENGTH - 1)  ; YL % RX_BUFFER_LENGTH
  sts rx_buffer_head_, YL  ; Save the value in YL to &rx_buffer_head_

  ; rx_buffer_[rx_buffer_head_] = ADC;
  lds r1, UDR0  ; Load the received byte in into r1
  clr YH
  subi YL, lo8(-(rx_buffer_))  ; Add &rx_buffer_[0] to rx_buffer_head_
  sbci YH, hi8(-(rx_buffer_))  ; Add &rx_buffer_[0] to rx_buffer_head_
  st Y, r1  ; Put the byte in r1 to the SRAM address in Y

  ; Restore the state of SREG
  out __SREG__, r0

  ; Restore the state of the freed registers (in order).
  pop YH  ; Restore YH (R28) from the stack
  pop YL  ; Restore YL (R28) from the stack
  pop r1  ; Restore r1 from the stack
  pop r0
  reti
