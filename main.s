#include <xc.inc>

ANCON3 equ 0xF4A

global ACCEL_X_L, ACCEL_X_H, ACCEL_Y_L, ACCEL_Y_H
extrn  Setup_Accel, Read_Accel
extrn  DAC_Setup, DAC_Int_Hi

psect udata_acs
ACCEL_X_L:  ds 1
ACCEL_X_H:  ds 1
ACCEL_Y_L:  ds 1
ACCEL_Y_H:  ds 1
TEMP_H:     ds 1
TEMP_L:     ds 1
DEFAULT_H:  ds 1
DEFAULT_L:  ds 1
counter:    ds 1

psect code, abs
rst:    org 0x0000
        goto start

int_hi: org 0x0008          ; high priority interrupt vector
        goto DAC_Int_Hi

start:
    ; 16MHz internal oscillator
    banksel OSCCON
    movlw 0x70
    movwf OSCCON, b

    ; Disable all analog inputs
    banksel ANCON3
    clrf ANCON3, b
    banksel ANCON0
    clrf ANCON0, b
    clrf ANCON1, b
    clrf ANCON2, b

    ; Port E - output (X accel display)
    clrf TRISE, A
    clrf LATE, A

    ; Port F - output (Y accel display)
    clrf TRISF, A
    clrf LATF, A

    ; Port J - output (debug counter)
    clrf TRISJ, A
    clrf LATJ, A

    ; Port D - direction set by Setup_Accel, zero lats first
    clrf TRISD, A
    clrf LATD, A

    call Setup_Accel
    call DAC_Setup

    ; Centre position PWM value = 1.5ms = 0x6F66
    movlw 0x6F
    movwf DEFAULT_H, c

    movlw 0x66
    movwf DEFAULT_L, c

loop:
    call Read_Accel
    
    movlw 0x80
    movwf ACCEL_X_H, A
    movlw 0x00
    movwf ACCEL_X_L, A

    ; Convert X and drive motor X (RD2)
    movff ACCEL_X_L, TEMP_L
    movff ACCEL_X_H, TEMP_H
    call convert_tilt
    call move_motor_X

    ; Convert Y and drive motor Y (RD3)
    movff ACCEL_Y_L, TEMP_L
    movff ACCEL_Y_H, TEMP_H
    call convert_tilt
    call move_motor_Y

    ; Display raw high bytes on LEDs
    movf ACCEL_X_H, W, c
    movwf LATE, A

    movf ACCEL_Y_H, W, c
    movwf LATF, A

    bra loop

; --- Motor control ---
; The interrupt sets RD2/RD3 high at the start of each 20ms cycle.
; move_motor_X/Y clear the pin once the timer exceeds the pulse width
; stored in TEMP_H:TEMP_L by convert_tilt.
;
; 16-bit comparison: check high byte first.
; If TMR0H > TEMP_H  -> clear pin immediately
; If TMR0H == TEMP_H -> check low byte
; If TMR0H < TEMP_H  -> not yet, return

move_motor_X:
    movf TEMP_H, W, c
    cpfsgt TMR0H            ; skip if TMR0H > TEMP_H
    bra check_low_X
    bcf LATD, 2, A          ; TMR0H > TEMP_H, clear pin now
    return
check_low_X:
    cpfseq TMR0H            ; skip if TMR0H == TEMP_H
    return                  ; TMR0H < TEMP_H, not yet
    movf TEMP_L, W, c
    cpfsgt TMR0L            ; skip if TMR0L > TEMP_L
    return
    bcf LATD, 2, A
    return

move_motor_Y:
    movf TEMP_H, W, c
    cpfsgt TMR0H
    bra check_low_Y
    bcf LATD, 3, A
    return
check_low_Y:
    cpfseq TMR0H
    return
    movf TEMP_L, W, c
    cpfsgt TMR0L
    return
    bcf LATD, 3, A
    return

; --- Convert tilt to pulse width ---
; Arithmetic right-shifts TEMP_H:TEMP_L by 4 (divide by 16)
; then adds DEFAULT_H:DEFAULT_L (centre position = 0x6F66)
; Result is a timer compare value for the pulse width

convert_tilt:
    movlw 0x04
    movwf counter, c
shift_loop:
    ; Preserve sign bit through shift
    bcf  STATUS, 0, A
    btfsc TEMP_H, 7, c
    bsf  STATUS, 0, A

    rrcf TEMP_H, F, c
    rrcf TEMP_L, F, c

    decfsz counter, F, c
    bra shift_loop

    ; Add centre offset - low byte first so carry propagates correctly
    movf DEFAULT_L, W, c
    addwf TEMP_L, F, c

    movf DEFAULT_H, W, c
    addwfc TEMP_H, F, c

    return

    end rst