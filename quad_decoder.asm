;*******************************************************************************
;  Copyright Cambridge Silicon Radio Limited 2012-2014
;  Part of CSR uEnergy SDK 2.3.0
;  Application version 2.3.0.0
;
;  FILE
;      quad_decoder.asm
;
;  DESCRIPTION
;      8051 implementation of quadrature decoder.
;
;******************************************************************************/

; quadrature decoder inputs
.equ    QINPUT  ,P1             ; ZB = PIO10 (P1.2) and ZA = PIO11 (P1.3)
.equ    QMASK   ,0x0C           ; PIO mask (P1.2 and P1.3)
.equ    QZAMASK ,0x08           ; ZA PIO mask (P1.3)

.equ    Q10MASK ,0x08
.equ    Q01MASK ,0x04
.equ    Q00MASK ,0x00
.equ    Q11MASK ,0x0c

.equ    QUAD_BASE_VALUE, 0x7F   ; counter base value
.equ    DEBOUNCE_COUNT, 3

; states
.equ    STATE_00,         0
.equ    STATE_01_FROM_00, 1
.equ    STATE_01_FROM_11, 2
.equ    STATE_11,         3
.equ    STATE_10_FROM_00, 4
.equ    STATE_10_FROM_11, 5

; shared memory variables
.equ    COUNTER ,0x40
.equ    INT_ENABLE,     0x42
.equ    RESET_COUNTER,  0x44

; registers
.equ    EXT_INTERRUPT,  0x9E   ; Interrupt main XAP system with transition from 1 to 0.

START:
        mov             SP, #30H                ; set the stack up
        mov             R2, #0                  ; previous state
        mov             R1, #0                  ; temporary holder
        mov             R0, #COUNTER
        mov             @R0, #QUAD_BASE_VALUE   ; reset the counter
        mov             R5, #STATE_00           ; set the initial state
        
        mov             INT_ENABLE, #0
        mov             RESET_COUNTER, #0

; main loop             
QUAD_LOOP:
        
        ; check for counter reset flag
        mov             A, RESET_COUNTER
        jz              DEBOUNCE_LOOP

        mov             COUNTER, #QUAD_BASE_VALUE
        mov             RESET_COUNTER, #0

; debounce the input signals
DEBOUNCE_LOOP:
        mov             R3, #DEBOUNCE_COUNT

        mov             A, QINPUT       ; read current input
        anl             A, #QMASK       ; get input bits change
        mov             R4, A           ; store the value

COMPARE_LOOP:
        mov             A, QINPUT       ; read current input
        anl             A, #QMASK       ; get input bits change
        xrl             A, R4           ; diff with the previous value
        jnz             DEBOUNCE_LOOP   ; reset the reading

        dec             R3
        cjne            R3, #0, COMPARE_LOOP
        
        mov             A, R4           ; get debounced value
        xrl             A, R2           ; diff with previous
        jz              QUAD_LOOP       ; no change
        
        xrl             A, #QMASK       ; both inputs changed
        jnz              CONTINUE_CHECK   ; continue
        ljmp CONTINUE_LOOP
        
CONTINUE_CHECK:

        mov             A, R4           ; get debounced value

        cjne            R5, #STATE_00, STATE_SELECT_1
; *******************
; * STATE_00
; *******************
        xrl             A, #Q01MASK
        jz              STATE_00_INPUT_01

        mov             A, R4           ; get debounced value
        xrl             A, #Q10MASK
        jnz             CONTINUE_LOOP

STATE_00_INPUT_10:
; * input signal 10
        ; STATE_00 - input 10 -> STATE_10_FROM_00
        mov             R5, #STATE_10_FROM_00
        ajmp            CONTINUE_LOOP

STATE_00_INPUT_01:
; * input signal 01
        ; STATE_00 - input 01 -> STATE_01_FROM_00
        mov             R5, #STATE_01_FROM_00
        ajmp            CONTINUE_LOOP

STATE_SELECT_1:
        cjne            R5, #STATE_11, STATE_SELECT_2
; *******************
; * STATE_11
; *******************
        xrl             A, #Q01MASK
        jz              STATE_11_INPUT_01

        mov             A, R4           ; get debounced value
        xrl             A, #Q10MASK
        jnz              CONTINUE_LOOP

STATE_11_INPUT_10:
; * input signal 10
        ; STATE_11 - input 10 -> STATE_10_FROM_11
        mov             R5, #STATE_10_FROM_11
        ajmp            CONTINUE_LOOP

STATE_11_INPUT_01:
; * input signal 01
        ; STATE_11 - input 01 -> STATE_01_FROM_11
        mov             R5, #STATE_01_FROM_11
        ajmp            CONTINUE_LOOP

STATE_SELECT_2:
        cjne            R5, #STATE_10_FROM_00, STATE_SELECT_3
; *******************
; * STATE_10_FROM_00
; *******************
        xrl             A, #Q11MASK
        jz              STATE_10_FROM_00_INPUT_11

        mov             A, R4           ; get debounced value
        xrl             A, #Q00MASK
        jnz             CONTINUE_LOOP

STATE_10_FROM_00_INPUT_00:
; * input signal 00
        ; STATE_10_FROM_00 - input 00 -> STATE_00
        mov             R5, #STATE_00
        ajmp            CONTINUE_LOOP

STATE_10_FROM_00_INPUT_11:
; * input signal 11
        ; STATE_10_FROM_00 - input 11 -> STATE_11 output +1
        mov             R5, #STATE_11
        dec             @R0
        ajmp            INTERRUPT_XAP

STATE_SELECT_3:
        cjne            R5, #STATE_10_FROM_11, STATE_SELECT_4
; *******************
; * STATE_10_FROM_11
; *******************
        xrl             A, #Q11MASK
        jz              STATE_10_FROM_11_INPUT_11

        mov             A, R4           ; get debounced value
        xrl             A, #Q00MASK
        jnz              CONTINUE_LOOP

STATE_10_FROM_11_INPUT_00:
; * input signal 00
        ; STATE_10_FROM_11 - input 00 -> STATE_00 output -1
        mov             R5, #STATE_00
        inc             @R0
        ajmp            INTERRUPT_XAP

STATE_10_FROM_11_INPUT_11:
; * input signal 11
        ; STATE_10_FROM_11 - input 11 -> STATE_11
        mov             R5, #STATE_11
        ajmp            CONTINUE_LOOP
        
STATE_SELECT_4:
        cjne            R5, #STATE_01_FROM_00, STATE_SELECT_5
; *******************
; * STATE_01_FROM_00
; *******************
        xrl             A, #Q11MASK
        jz              STATE_01_FROM_00_INPUT_11
        
        mov             A, R4           ; get debounced value
        xrl             A, #Q00MASK
        jnz              CONTINUE_LOOP

STATE_01_FROM_00_INPUT_00:
; * input signal 00
        ; STATE_01_FROM_00 - input 00 -> STATE_00
        mov             R5, #STATE_00
        ajmp            CONTINUE_LOOP
        
STATE_01_FROM_00_INPUT_11:
; * input signal 11
        ; STATE_01_FROM_00 - input 11 -> STATE_11 output -1
        mov             R5, #STATE_11
        inc             @R0             
        ajmp            INTERRUPT_XAP

STATE_SELECT_5:
; *******************
; * STATE_01_FROM_11
; *******************
        xrl             A, #Q11MASK
        jz              STATE_01_FROM_11_INPUT_11

        mov             A, R4           ; get debounced value
        xrl             A, #Q00MASK
        jnz              CONTINUE_LOOP
                
STATE_01_FROM_11_INPUT_00:
; * input signal 00
        ; STATE_01_FROM_11 - input 00 -> STATE_00 output -1
        mov             R5, #STATE_00
        dec             @R0
        ajmp            INTERRUPT_XAP

STATE_01_FROM_11_INPUT_11:
; * input signal 11
        ; STATE_01_FROM_11 - input 11 -> STATE_11
        mov             R5, #STATE_11
        ajmp            CONTINUE_LOOP

INTERRUPT_XAP:
        mov             A, INT_ENABLE       ; read the INT_ENABLE variable
        jz              CONTINUE_LOOP

        ; trigger the external interrupt, if enabled
        mov             EXT_INTERRUPT, #1
        mov             EXT_INTERRUPT, #0

CONTINUE_LOOP:
; save the previous state and continue scan             
        mov             A, R4
        mov             R2, A
        ljmp    QUAD_LOOP               
; END

