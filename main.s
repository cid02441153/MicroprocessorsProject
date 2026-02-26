#include <xc.inc>

extrn	DAC_Setup, DAC_Int_Hi
    
psect data_acs
delay_count: ds 1
delay_count_2: ds 1
delay_count_3: ds 1
ACCEL_X_L: ds 1
ACCEL_X_H: ds 1
ACCEL_Y_L: ds 1
ACCEL_Y_H: ds 1
TEMP_H: ds 1
TEMP_L: ds 1
DEFAULT_H: ds 1
DEFAULT_L: ds 1
counter:    ds 1
    
psect	code, abs
rst:	org	0x0000	; reset vector
	goto	start

int_hi:	org	0x0008	; high vector, no low vector
	goto	DAC_Int_Hi
	
start:	
    call	DAC_Setup
    
    movlw 0x6F
    movwf DEFAULT_H, A
    
    movlw 0x66
    movwf DEFAULT_L, A
    
loop: 
    ; 'read in' accelerometer value
    ; at the moment hard coded into ACCEL_X_H, ACCEL_X_L
    movlw 0x7f
    movwf TEMP_H
    
    movlw 0xff
    movwf TEMP_L
    
    call convert_tilt ; divide value by 16
    
    ; Store into appropriate X variables
    movff TEMP_H, ACCEL_X_H
    movff TEMP_L, ACCEL_X_L
    
    ; Y tilt
    movlw 0x80
    movwf TEMP_H
    
    movlw 0x00
    movwf TEMP_L
   
    call convert_tilt ; divide value by 16
    
    ; Store into appropriate Y variables
    movff TEMP_H, ACCEL_Y_H
    movff TEMP_L, ACCEL_Y_L
    
    ; Move Motor
    call move_motor_X
    call move_motor_Y
    
    ;call move_left
    ;call move_centre
    ;call move_right
   
    bra loop 
    
move_motor_X:
    movf ACCEL_X_L, W
    cpfsgt TMR0L
    return
    
    movf ACCEL_X_H, W
    cpfsgt  TMR0H ; compare w and f, skip if greater than 
    return
    
    bcf PORTD, 2, A
    return

move_motor_Y:
    movf ACCEL_Y_L, W
    cpfsgt TMR0L
    return
    
    movf ACCEL_Y_H, W
    cpfsgt  TMR0H ; compare w and f, skip if greater than 
    return
    
    bcf PORTD, 3, A
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
    
    bcf LATD, 2, A
    return
    
move_right: ; 0.5ms - 0x6794
    movlw 0x94
    cpfsgt TMR0L
    return
    
    movlw 0x67
    cpfsgt  TMR0H ; compare w and f, skip if greater than 
    return
    
    bcf LATD, 2, A
    return

; Divides value by 16 to be in range of servo
convert_tilt:
    movlw 0x04
    movwf counter
shift_loop:
    bcf	STATUS, 0
    btfsc TEMP_H, 7
    bsf	STATUS, 0
    
    ; Rotate high byte
    rrcf TEMP_H, F
    
    ; Rotate low byte
    rrcf TEMP_L, F
    
    ; Repeat 4 times for a shift of 4
    decfsz counter, F
    bra shift_loop
    
    ; Add default value
    movf DEFAULT_H, W, A
    addwf TEMP_H, F
    
    movf DEFAULT_L, W, A
    addwfc TEMP_L, F
    
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