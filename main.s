#include <xc.inc>
    
global ACCEL_X_H, ACCEL_X_L, ACCEL_Y_H, ACCEL_Y_L
global TARGET_X_H, TARGET_X_L, TARGET_Y_H, TARGET_Y_L
global GYRO_X_L, GYRO_X_H, GYRO_Y_L, GYRO_Y_H
global MATH_FLAG
    
extrn	DAC_Setup, DAC_Int_Hi
extrn	Setup_Accel, Read_Accel, Read_Gyro
extrn	UART_Setup, UART_Transmit_Message
extrn   move_motor_X, move_motor_Y
    
psect udata_acs
SHIFT_H: ds 1
SHIFT_L: ds 1
    
delay_count: ds 1
delay_count_2: ds 1
delay_count_3: ds 1
    
; Variables for PWM signal to compare to timer0
TARGET_X_L: ds 1
TARGET_X_H: ds 1
TARGET_Y_L: ds 1
TARGET_Y_H: ds 1

; Tilt values (distance from 0) for potential gain
ACCEL_X_L: ds 1
ACCEL_X_H: ds 1
ACCEL_Y_L: ds 1
ACCEL_Y_H: ds 1
    
; Total error (cumulative distance from 0) for integral gain
TOTAL_ERROR_X_L: ds 1 
TOTAL_ERROR_X_H: ds 1     
TOTAL_ERROR_Y_L: ds 1 
TOTAL_ERROR_Y_H: ds 1 
    
; Gyro values (speed of rotation) for derivative gain
GYRO_X_L: ds 1
GYRO_X_H: ds 1
GYRO_Y_L: ds 1
GYRO_Y_H: ds 1
    
; Temp variables so that routines can be used for both X and Y
TEMP_H: ds 1
TEMP_L: ds 1
    
; Flag for interrupt
MATH_FLAG: ds 1
    
counter: ds 1
    
; PID Gain Values    
K_p: ds 1
K_d: ds 1
K_i: ds 1
    
psect	code, abs
rst:	org	0x0000	; reset vector
	goto	start

int_hi:	org	0x0008	; high vector, no low vector
	goto	DAC_Int_Hi
	
start:
    
    ; Set Gains
    movlw 0x06
    movwf K_p
    
    movlw 0x0B
    movwf K_d
    
    movlw 0x07
    movwf K_i
    
    ; Initial target is 0 degrees
    movlw 0x6F
    movwf TARGET_X_H, A
    
    movlw 0x65
    movwf TARGET_X_L, A
    
    ; Initial target is 0 degrees
    movlw 0x6F
    movwf TARGET_Y_H, A
    
    movlw 0x65
    movwf TARGET_Y_L, A
    
    ; --- INITIALISE ERROR ---
    movlw 0x6F
    movwf TOTAL_ERROR_X_H, A
    movlw 0x65
    movwf TOTAL_ERROR_X_L, A
    
    movlw 0x6F
    movwf TOTAL_ERROR_Y_H, A
    movlw 0x65
    movwf TOTAL_ERROR_Y_L, A
   
    ; Clear Port D for Output
    clrf TRISD, A
    clrf LATD
    
    ; Setup
    call	UART_Setup
    call	Setup_Accel
    call	DAC_Setup
    call	delay
    
loop: 
    
    ; Checks the timer for the motors EVERY LOOP
    call move_motor_X
    call move_motor_Y
    
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
    
    ; ==============================
    ; X-AXIS PID CALCULATIONS
    ; ==============================

    ; --- INTEGRAL GAIN X ---
    movff ACCEL_X_H, TEMP_H
    movff ACCEL_X_L, TEMP_L
    movf K_i, W, A 
    call convert_tilt

    ; Subtract the converted accelerometer reading
    ; Subtract = move motor in opposite direction to the error
    movf TEMP_L, W, A
    subwf TOTAL_ERROR_X_L, F, A
    movf TEMP_H, W, A
    subwfb TOTAL_ERROR_X_H, F, A
    
    movlw 0x80
    cpfslt TOTAL_ERROR_X_H, A 
    bra clamp_max_X
    
    movlw 0x60
    cpfsgt TOTAL_ERROR_X_H, A 
    bra clamp_min_X
    
    bra clamp_done_X
    
clamp_max_X:
    movlw 0x75
    movwf TOTAL_ERROR_X_H, A
    movlw 0x00
    movwf TOTAL_ERROR_X_L, A
    bra clamp_done_X
    
clamp_min_X:
    movlw 0x65
    movwf TOTAL_ERROR_X_H, A
    movlw 0x00
    movwf TOTAL_ERROR_X_L, A
    bra clamp_done_X
    
clamp_done_X:
    ; --- PROPORTIONAL GAIN X ---
    movff ACCEL_X_H, TEMP_H
    movff ACCEL_X_L, TEMP_L
    movf K_p, W, A 
    call convert_tilt

    ; Subtract the proportional term from the total error
    movf TEMP_L, W, A
    subwf TOTAL_ERROR_X_L, W, A
    movwf TARGET_X_L, A 
    movf TEMP_H, W, A
    subwfb TOTAL_ERROR_X_H, W, A
    movwf TARGET_X_H, A 
    
    ; --- DERIVATIVE GAIN X ---
    movff GYRO_X_H, TEMP_H
    movff GYRO_X_L, TEMP_L
    movf K_d, W, A
    call convert_tilt

    ; Add the gyro damping term
    movf TEMP_L, W, A
    addwf TARGET_X_L, F, A
    movf TEMP_H, W, A
    addwfc TARGET_X_H, F, A
    
    
    ; ==============================
    ; Y-AXIS PID CALCULATIONS
    ; ==============================

    ; --- INTEGRAL GAIN Y ---
    movff ACCEL_Y_H, TEMP_H
    movff ACCEL_Y_L, TEMP_L
    
    movf K_i, W, A 
    call convert_tilt

    ; Subtract the converted accelerometer reading
    movf TEMP_L, W, A
    addwf TOTAL_ERROR_Y_L, F, A
  
    movf TEMP_H, W, A
    addwfc TOTAL_ERROR_Y_H, F, A
    
    movlw 0x80 
    cpfslt TOTAL_ERROR_Y_H, A 
    bra clamp_max_Y
    
    movlw 0x60
    cpfsgt TOTAL_ERROR_Y_H, A 
    bra clamp_min_Y
    
    bra clamp_done_Y
    
clamp_max_Y:
    movlw 0x75
    movwf TOTAL_ERROR_Y_H, A
    movlw 0x00
    movwf TOTAL_ERROR_Y_L, A
    bra clamp_done_Y
    
clamp_min_Y:
    movlw 0x55
    movwf TOTAL_ERROR_Y_H, A
    movlw 0x00
    movwf TOTAL_ERROR_Y_L, A
    bra clamp_done_Y
    
clamp_done_Y:   
    
    ; --- PROPORTIONAL GAIN Y ---
    movff ACCEL_Y_H, TEMP_H
    movff ACCEL_Y_L, TEMP_L
    movf K_p, W, A
    call convert_tilt

    ; Subtract the proportional term from the total error
    movf TEMP_L, W, A
    addwf TOTAL_ERROR_Y_L, W, A
    movwf TARGET_Y_L, A 
    movf TEMP_H, W, A
    addwfc TOTAL_ERROR_Y_H, W, A
    movwf TARGET_Y_H, A 
    
    ; --- DERIVATIVE GAIN Y ---
    movff GYRO_Y_H, TEMP_H
    movff GYRO_Y_L, TEMP_L
    movf K_d, W, A
    call convert_tilt

    ; Add the gyro damping term
    movf TEMP_L, W, A
    subwf TARGET_Y_L, F, A
    movf TEMP_H, W, A
    subwfb TARGET_Y_H, F, A

    ; Infinite loop
    bra loop
    

; Set an initial angle for the servo for testing
init_angle:
    movlw 0x80
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
    call right_shift_W
    movff SHIFT_H, TEMP_H
    movff SHIFT_L, TEMP_L
    
    return
    
right_shift_W: ; Amount is stored in W
    movwf counter, A
    movf counter, F, a
    bz shift_zero
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
    
shift_zero:
    ; Gain = 0, so set shift variables to zero
    movlw 0x00
    movwf SHIFT_H, A
    movwf SHIFT_L, A
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