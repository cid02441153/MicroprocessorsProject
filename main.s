#include <xc.inc>

global ACCEL_X_H, ACCEL_X_L, ACCEL_Y_H, ACCEL_Y_L    
    
extrn	DAC_Setup, DAC_Int_Hi
extrn	Setup_Accel, Read_Accel
extrn	UART_Setup, UART_Transmit_Message
    
psect udata_acs
delay_count: ds 1
delay_count_2: ds 1
delay_count_3: ds 1
ACCEL_X_L: ds 1
ACCEL_X_H: ds 1
ACCEL_Y_L: ds 1
ACCEL_Y_H: ds 1
AVERAGE_L: ds 1
AVERAGE_H: ds 1
TEMP_H: ds 1
TEMP_L: ds 1
DEFAULT_H: ds 1
DEFAULT_L: ds 1
counter:    ds 1
UART_counter: ds 1
    
psect	code, abs
rst:	org	0x0000	; reset vector
	goto	start

int_hi:	org	0x0008	; high vector, no low vector
	goto	DAC_Int_Hi
	
start:
    
    movlw 0x00
    movwf AVERAGE_L, A
    movwf AVERAGE_H, A
    
    movlw 0x6F
    movwf DEFAULT_H, A
    
    movlw 0x66
    movwf DEFAULT_L, A
    
    movlw 0xFF
    movwf UART_counter, A
    
    clrf TRISE, A
    clrf LATE
    
    clrf TRISF, A
    clrf LATF
    
    clrf TRISD, A
    clrf LATD
    
    call	UART_Setup
    call	Setup_Accel
    call	DAC_Setup
   
    
loop:
    call Read_Accel

    decfsz UART_counter, A
    bra skip_UART
    
    movlw 0xFF
    movwf UART_counter
    
    lfsr 2, ACCEL_X_H
    movlw 0x01
    call UART_Transmit_Message
    
skip_UART:
    ; Load new X sample into TEMP
    movff ACCEL_X_H, TEMP_H
    movff ACCEL_X_L, TEMP_L

    ; EMA: TEMP = TEMP - AVERAGE
    movf AVERAGE_L, W, A
    subwf TEMP_L, F
    movf AVERAGE_H, W, A
    subwfb TEMP_H, F

    ; Divide by 16
    movlw 0x09
    call right_shift_W

    ; AVERAGE = AVERAGE + TEMP
    movf TEMP_L, W, A
    addwf AVERAGE_L, F, A
    movf TEMP_H, W, A
    addwfc AVERAGE_H, F, A

    ; Use filtered average as input to convert_tilt
    movff AVERAGE_H, TEMP_H
    movff AVERAGE_L, TEMP_L

    call convert_tilt

    movff TEMP_H, ACCEL_X_H
    movff TEMP_L, ACCEL_X_L

    ; Same for Y
    movff ACCEL_Y_H, TEMP_H
    movff ACCEL_Y_L, TEMP_L
    call convert_tilt
    movff TEMP_H, ACCEL_Y_H
    movff TEMP_L, ACCEL_Y_L

    call move_motor_X
    call move_motor_Y

    bra loop
    
move_motor_X:
    
    movf ACCEL_X_L, W, A
    cpfsgt TMR0L
    return
    
    movf ACCEL_X_H, W
    cpfsgt  TMR0H ; compare w and f, skip if greater than 
    return
    
    bcf PORTD, 2, A
    return

move_motor_Y:
    movf ACCEL_Y_L, W, A
    movf ACCEL_Y_L, W
    cpfsgt TMR0L
    return
    
    movf ACCEL_Y_H, W
    cpfsgt  TMR0H ; compare w and f, skip if greater than 
    return
    
    bcf PORTD, 3, A
    return

; Divides value by 16 to be in range of servo
convert_tilt:
    movlw 0x03
    call right_shift_W
    
    movf DEFAULT_H, W, A
    addwf TEMP_H, F

    movf DEFAULT_L, W, A
    addwfc TEMP_L, w
    
    return
    
right_shift_W: ; Amount is stored in W
    movwf counter
shift_loop:
    bcf	STATUS, 0
    btfsc TEMP_H, 7
    bsf	STATUS, 0
    
    ; Rotate high byte
    rrcf TEMP_H, F
    
    ; Rotate low byte
    rrcf TEMP_L, F
    
    ; Repeat 3 times for a shift of 3
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