#include <xc.inc>

extrn	DAC_Setup, DAC_Int_Hi
    
psect udata_acs
delay_count: ds 1
delay_count_2: ds 1
delay_count_3: ds 1
ACCEL_X_L: ds 1
ACCEL_X_H: ds 1
counter:    ds 1
    
psect	code, abs
rst:	org	0x0000	; reset vector
	goto	start

int_hi:	org	0x0008	; high vector, no low vector
	goto	DAC_Int_Hi
	
start:	
    call	DAC_Setup
    
loop: 
    ; 'read in' accelerometer value
    ; at the moment hard coded into ACCEL_X_H, ACCEL_X_L
    movlw 0x7F
    movwf ACCEL_X_H
    
    movlw 0xFF
    movwf ACCEL_X_L
    
    call right_shift_4 ; divide value by 16
    
    ; Add default value
    movlw 0x6F
    addwf ACCEL_X_H, F
    
    movlw 0x66
    addwfc ACCEL_X_L, F
    
    ; Move Motor
    ;call move_motor
    
    ;call move_left
    ;call move_centre
    ;call move_right
   
    bra loop 
    
move_motor:
    ; High byte first
    movf ACCEL_X_H, W
    cpfsgt TMR0H            ; skip if TMR0H > ACCEL_X_H
    bra check_low
    bcf PORTD, 2, A         ; TMR0H > ACCEL_X_H, clear pin
    return
check_low:
    cpfseq TMR0H            ; skip if TMR0H == ACCEL_X_H
    return                  ; TMR0H < ACCEL_X_H, not yet
    movf ACCEL_X_L, W
    cpfsgt TMR0L            ; skip if TMR0L > ACCEL_X_L
    return
    bcf PORTD, 2, A
    return
    
    
move_left: ; 2.5ms - 0x7737
    movlw 0x37
    cpfsgt TMR0L
    return
    
    movlw 0x77
    cpfsgt  TMR0H ; compare w and f, skip if greater than 
    return
    
    bcf PORTD, 2, A
    return
    
move_centre: ; 1.5ms - 0x6F66
    movlw 0x66
    cpfsgt TMR0L
    return
    
    movlw 0x6F
    cpfsgt  TMR0H ; compare w and f, skip if greater than 
    return
    
    bcf PORTD, 2, A
    return
    
move_right: ; 0.5ms - 0x6794
    movlw 0x94
    cpfsgt TMR0L
    return
    
    movlw 0x67
    cpfsgt  TMR0H ; compare w and f, skip if greater than 
    return
    
    bcf PORTD, 2, A
    return

; Divides value by 16 to be in range of servo
right_shift_4:
    movlw 0x04
    movwf counter
shift_loop:
    bcf	STATUS, 0
    btfsc ACCEL_X_H, 7
    bsf	STATUS, 0
    
    ; Rotate high byte
    rrcf ACCEL_X_H, F
    
    ; Rotate low byte
    rrcf ACCEL_X_L, F
    
    ; Repeat 4 times for a shift of 4
    decfsz counter, F
    bra shift_loop
    return
    
delay:
    movlw 0xFF
    movwf delay_count
delayouter:
    movlw 0xFF
    movwf delay_count_2
delayinner_1:
    movlw 0xFF
    movwf delay_count_3
delayinner_2:
    decfsz delay_count_3, F
    bra delayinner_2
    
    decfsz delay_count_2, F
    bra delayinner_1
    
    decfsz delay_count, F
    bra delayouter
    
    return

	end	rst