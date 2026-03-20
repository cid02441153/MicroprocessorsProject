#include <xc.inc>

global MATH_FLAG
    
extrn	DAC_Setup, DAC_Int_Hi
extrn	Setup_Accel, Read_Accel, Read_Gyro
extrn	UART_Setup, UART_Transmit_Message
extrn   move_motor_X, move_motor_Y, init_angle
extrn	stabilise_motors, initialise_PID
extrn	ACCEL_X_H, ACCEL_Y_H
extrn	GYRO_X_H, GYRO_Y_H
extrn	FILTERED_X_H, FILTERED_Y_H
extrn	TARGET_X_H, TARGET_Y_H
    
psect udata_acs
 
delay_count: ds 1
delay_count_2: ds 1
delay_count_3: ds 1
    
; Flag for interrupt
MATH_FLAG: ds 1
    
    
psect	code, abs
rst:	org	0x0000	; reset vector
	goto	start

int_hi:	org	0x0008	; high vector, no low vector
	goto	DAC_Int_Hi
	
start:
   
    ; Clear Port D for Output
    clrf	TRISD, A
    clrf	LATD, A
    
    ; Setup
    call	initialise_PID
    call	UART_Setup
    call	Setup_Accel
    call	DAC_Setup
    call	delay
    
loop: 
    
    
    ; Checks the timer for the motors EVERY LOOP
    
    btfsc PORTD, 2
    call move_motor_X
    
    btfsc PORTD, 3
    call move_motor_Y
    
    ;call init_angle
    ;bra loop 
   
    ; Only does the accelerometer maths once per interrupt (20ms cycle)
    ; MATH_FLAG gets set to one each time timer0 interrupts
    btfss MATH_FLAG, 0
    bra loop
    bcf MATH_FLAG, 0
    
    ; Read the X and Y accelerometer readings
    call Read_Accel
    call Read_Gyro
    
    ; Write to UART
    lfsr 2, ACCEL_X_H
    movlw 0x01
    call UART_Transmit_Message
    
    lfsr 2, ACCEL_Y_H
    movlw 0x01
    call UART_Transmit_Message
    
    ; Convert the tilt values to timer comparison values
    call stabilise_motors
    ;call init_angle
    
    ; Infinite loop
    bra loop
    
    
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