; This file provides an interrupt handler for reception of a Futaba SBus message
; on USART1. This function uses a double buffer so that reading and writing can
; be done simultaneously. This function begins immediately writing to a buffer
; when a new byte is received, but resets if a delay of more than 1ms occurs
; between receptions (contiguous bytes should arrive every 0.12 ms). Reception
; also resets if any error is detected (frame, overrun, or parity errors). The
; reception is considered complete when 25 contiguous bytes have been received.
; One this happens the timestamp is also appended to the buffer and the variable
; sbus_data_ready_ is loaded with which of of the double buffers the data
; resides in. The code that parses this buffer should check for validity of the
; start and end bytes, as well as the signal-lost and fail-safe flags.

; Stack usage: 4 bytes
; Worst case runtime: 75 cycles

; The following references were very helpful in making this file:
; 8-bit AVR Instruction Set
; Atmel AT1886: Mixing Assembly and C with AVRGCC (has many errors though)
; ATmega164A/PA/324A/PA/644A/PA/1284/P Datasheet (Instruction Set Summary)

; Encapsulating the include in a .nolist statement prevents a bunch of
; unnecessary output in the .lst file.
.nolist
#include "sbus.h"
#include <avr/io.h>
.list

.extern ms_timestamp_  ; int16_t
.extern sbus_rx_buffer_  ; uint8_t[2][SBUS_MESSAGE_LENGTH+2]
.extern sbus_data_ready_  ; uint8_t

__SREG__ = _SFR_IO_ADDR(SREG)


; Static variable declarations
;;
.section .bss.last_rx_timestamp,"aw",@nobits
  last_rx_timestamp:
  .zero 1
.section .bss.rx_counter,"aw",@nobits
  rx_counter:
  .zero 1
.section .bss.rx_buffer_n,"aw",@nobits
  rx_buffer_n:
  .zero 1


; This interrupt handler places 
;;
.section .text.USART1_RX_vect,"ax",@progbits
.global USART1_RX_vect
USART1_RX_vect:
  ; Save the state of SREG to be restored before returning.
  in r1, __SREG__  ; Save SREG in R1 (normally reserved for 0)

  ; Free up some registers by pushing their contents to the stack.
  push r24  ; Save r24 to the stack
  push r25  ; Save r25 to the stack
  push XL  ; Save XL (r26) to the stack
  push XH  ; Save XH (r27) to the stack

  ; Load the current and previous (8-bit) ms timestamp.
  lds r26, ms_timestamp_  ; Load only the lower byte of ms_timestamp_ into r26
  lds r27, last_rx_timestamp  ; Load last Rx timestamp into r27
  sts last_rx_timestamp, r26  ; Save the timestamp for the next interrupt

  ; Reset if any Rx errors.
  lds r24, UCSR1A  ; Load USART1 status register into r24
  andi r24, ((FE1 << 1) | (DOR1 << 1) | (UPE1 << 1))  ; Check for Rx errors
  brne SBUS_RX_RESET  ; If r24 is not 0, then some Rx error occurred

  ; Check for ongoing reception (rx_counter != 0)
  lds r24, rx_counter  ; Load rx_counter into r24
  dec r24  ; r24--
  brmi SBUS_FIRST_BYTE  ; If r24 is negative, then it must be the first byte

  ; Reset if >1 ms has passed since last reception.
  sub r26, r27  ; r26 = r26 (ms_timestamp_) - r27 (last_rx_timestamp)
  andi r26, 0xFE  ; Clear the last bit of r26
  brne SBUS_RX_RESET  ; If r26 is not 0, then >1 ms must have passed

SBUS_STORE_BYTE:
  ; Get the buffer position.
  mov XL, r24  ; Copy r24 into XL (r26)
  clr XH  ; Set XH (r27) to 0
  lds r25, rx_buffer_n  ; Load rx_buffer_n into r25
  sbrc r25, 0  ; Skip next instruction if bit 0 in r25 is 0
  subi XL, -SBUS_MESSAGE_LENGTH  ; X = X - (-SBUS_MESSAGE_LENGTH)
  subi XL, lo8(-(sbus_rx_buffer_))  ; X = X - (-&sbus_rx_buffer_)
  sbci XH, hi8(-(sbus_rx_buffer_))  ; X = X - (-&sbus_rx_buffer_)

  ; Save rx_counter and remember if it is not zero.
  sts rx_counter, r24  ; Save r24 to rx_counter
  tst r24  ; Test r24 for 0 (and negative)
  breq .+2  ; Skip next instruction if above is 0
  sbr r25, 0x02  ; Set bit 1 in r25

  ; Save the incoming byte to the buffer.
  lds r24, UDR1  ; Load UDR1 into r24
  st X, r24  ; Save r24 to *X

  ; Return if rx_counter is not zero.
  sbrc r25, 1  ; Skip next instruction if bit 1 is set in r25
  rjmp SBUS_RX_RETURN

  ; Save timestamp to the buffer.
  lds r24, ms_timestamp_+1  ; Load upper byte of timestamp into r24
  st -X, r24  ; Save r24 to the buffer
  lds r24, ms_timestamp_  ; Load lower byte of timestamp into r24
  st -X, r24  ; Save r24 to the buffer

  ; Indicate that data is ready in the current rx_buffer.
  sts sbus_data_ready_, r25  ; Save r25 (current buffer)

  ; Switch buffers for next reception.
  ldi r24, 0x01  ; Load 0x01 into r24
  eor r25, r24  ; r25 = r25 ^ r24
  sts rx_buffer_n, r25  ; Save r25 to rx_buffer_n

SBUS_RX_RETURN:
  mov r24, r1  ; (because clr will set the Z flag)
  clr r1  ; Restore R1 to 0
  out __SREG__, r24  ; Restore the state of SREG

  ; Restore the state of the freed registers (in order).
  pop XH  ; Restore XH (r27) from the stack
  pop XL  ; Restore XL (r26) from the stack
  pop r25  ; Restore r25 from the stack
  pop r24  ; Restore r24 from the stack
  reti

SBUS_FIRST_BYTE:
  ldi r24, (SBUS_MESSAGE_LENGTH - 1)
  rjmp SBUS_STORE_BYTE

SBUS_RX_RESET:
  clr r24
  sts rx_counter, r24
  rjmp SBUS_RX_RETURN
