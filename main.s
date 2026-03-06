#include <xc.inc>
    
global ACCEL_X_H, ACCEL_X_L, ACCEL_Y_H, ACCEL_Y_L
global TARGET_X_H, TARGET_X_L, TARGET_Y_H, TARGET_Y_L
global MATH_FLAG
    
extrn	DAC_Setup, DAC_Int_Hi
extrn	Setup_Accel, Read_Accel
extrn	UART_Setup, UART_Transmit_Message
extrn   move_motor_X, move_motor_Y
    
psect udata_acs
SHIFT_H: ds 1
SHIFT_L: ds 1
    
delay_count: ds 1
delay_count_2: ds 1
delay_count_3: ds 1
    
; Variables for PWM signal
TARGET_X_L: ds 1
TARGET_X_H: ds 1
TARGET_Y_L: ds 1
TARGET_Y_H: ds 1

; Tilt values (distance from 0) for potential gain
ACCEL_X_L: ds 1
ACCEL_X_H: ds 1
ACCEL_Y_L: ds 1
ACCEL_Y_H: ds 1
TEMP_H: ds 1
TEMP_L: ds 1
    
; Flag for interrupt
MATH_FLAG: ds 1
counter: ds 1
    
    
psect	code, abs
rst:	org	0x0000	; reset vector
	goto	start

int_hi:	org	0x0008	; high vector, no low vector
	goto	DAC_Int_Hi
	
start:
    
    ; Initial target is 0 degrees
    movlw 0x6F
    movwf TARGET_X_H, A
    
    movlw 0x65
    movwf TARGET_X_L, A
   
    ; Clear Port D for Output
    clrf TRISD, A
    clrf LATD
    
    ; Setup
    call	UART_Setup
    call	Setup_Accel
    call	DAC_Setup
    
loop:
    
    ; COMMENT OUT FOR MAIN CODE
    ; UNCOMMENT FOR INTIIAL MOVEMENT
    ;call init_angle
    
    ; Checks the timer for the motors EVERY LOOP
    call move_motor_X
    call move_motor_Y
    
    ; Only does the accelerometer maths once per interrupt (20ms cycle)
    btfss MATH_FLAG, 0
    bra loop
    bcf MATH_FLAG, 0
    
    ; Read the X and Y accelerometer readings
    call Read_Accel
    
    ; Write to UART
    lfsr 2, ACCEL_X_H
    movlw 0x01
    call UART_Transmit_Message
    
    ; Convert X acceleration into motor servo PWM
    movff ACCEL_X_H, TEMP_H
    movff ACCEL_X_L, TEMP_L
    call convert_tilt
    movff TEMP_H, ACCEL_X_H
    movff TEMP_L, ACCEL_X_L

    ; Add the converted accelerometer reading to the target - INTEGRATION GAIN
    movf ACCEL_X_L, W, A
    addwf TARGET_X_L, F, A
    
    movf ACCEL_X_H, W, A
    addwfc TARGET_X_H, F, A
    
    ; Do the same for the Y servo
    movff ACCEL_Y_H, TEMP_H
    movff ACCEL_Y_L, TEMP_L
    call convert_tilt
    movff TEMP_H, ACCEL_Y_H
    movff TEMP_L, ACCEL_Y_L

    movf ACCEL_Y_L, W, A
    addwf TARGET_Y_L, F, A
    
    movf ACCEL_Y_H, W, A
    addwfc TARGET_Y_H, F, A

    ; Infinite loop
    bra loop
    

; Set an initial angle for the servo for testing
init_angle:
    movlw 0x69
    movwf TARGET_X_H, A
    
    movlw 0x00
    movwf TARGET_X_L, A
    
    call move_motor_X
    
    call init_angle
    
    return
    
; Divides value by 8 AND multiplies by the gain to be in range of servo
convert_tilt:
    
    ; Shift actual tilt values
    movff TEMP_H, SHIFT_H
    movff TEMP_L, SHIFT_L
    movlw 0x08 ; Right shift 3 for scale factor, right shift 5 for integration gain
    call right_shift_W
    movff SHIFT_H, TEMP_H
    movff SHIFT_L, TEMP_L
    
    return
    
right_shift_W: ; Amount is stored in W
    movwf counter
shift_loop:
    ; Clear carry bit
    bcf	STATUS, 0
    btfsc SHIFT_H, 7
    bsf	STATUS, 0
    
    ; Rotate high byte
    rrcf SHIFT_H, F
    
    ; Rotate low byte
    rrcf SHIFT_L, F
    
    ; Repeat 3 times for a shift of 3
    decfsz counter, F
    bra shift_loop
    
    return
    
; BIG delay
delay:
    movlw 0x5F
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