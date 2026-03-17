#include <xc.inc>
    
global ACCEL_X_H, ACCEL_X_L, ACCEL_Y_H, ACCEL_Y_L
global TARGET_X_H, TARGET_X_L, TARGET_Y_H, TARGET_Y_L
global GYRO_X_L, GYRO_X_H, GYRO_Y_L, GYRO_Y_H
global MATH_FLAG
    
extrn	DAC_Setup, DAC_Int_Hi
extrn	Setup_Accel, Read_Accel, Read_Gyro
extrn	UART_Setup, UART_Transmit_Message
extrn   move_motor_X, move_motor_Y, centre_motor_X
    
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
    movlw 0x05
    movwf K_p
    
    movlw 0x06
    movwf K_d
    
    movlw 0x09
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
    
    ; Initialize Total Error to act as the "Center" base PWM
    movlw 0x6F         ; X-Axis Center High Byte (based on your init code)
    movwf TOTAL_ERROR_X_H, A
    movlw 0x65          ; X-Axis Center Low Byte
    movwf TOTAL_ERROR_X_L, A

    movlw 0x6F          ; Y-Axis Center High Byte
    movwf TOTAL_ERROR_Y_H, A
    movlw 0x65          ; Y-Axis Center Low Byte
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

    ; =========================================
    ; --- ANTI-WINDUP LOGIC ---
    ; =========================================

    ; --- 1. CHECK CEILING (Max) ---
    movlw 0x77
    cpfslt TARGET_X_H, A     ; If Target < 0x76, ceiling is fine, check floor
    bra check_ceiling_sign_x   ; Target >= 0x76! Check if we should block it
    bra check_floor_x          ; Target is safe from ceiling, jump to floor check

check_ceiling_sign_x:
    btfss TEMP_H, 7          ; Is the error positive? (Bit 7 == 0)
    bra skip_x               ; YES: It will make saturation worse. SKIP MATH!
    bra do_integration_x       ; NO: It will help us recover. ALLOW MATH!

    ; --- 2. CHECK FLOOR (Min) ---
check_floor_x:
    movlw 0x68               ; (Using 0x68 as your floor limit)
    cpfsgt TARGET_X_H, A     ; If Target > 0x68, floor is fine, do math
    bra check_floor_sign_x    ; Target <= 0x68! Check if we should block it
    bra do_integration_x       ; Target is safe, proceed to math

check_floor_sign_x:
    btfsc TEMP_H, 7          ; Is the error negative? (Bit 7 == 1)
    bra skip_x               ; YES: It will make saturation worse. SKIP MATH!
    ; Fall through to do_integration

    ; =========================================
    ; --- INTEGRAL MATH ---
    ; =========================================
do_integration_x:
    movf TEMP_L, W, A
    addwf TOTAL_ERROR_X_L, F, A
    movf TEMP_H, W, A
    addwfc TOTAL_ERROR_X_H, F, A
skip_x:
   
    ; --- PROPORTIONAL GAIN X ---
    movff ACCEL_X_H, TEMP_H
    movff ACCEL_X_L, TEMP_L
    movf K_p, W, A 
    call convert_tilt

    ; Subtract the proportional term from the total error
    movf TEMP_L, W, A
    addwf TOTAL_ERROR_X_L, W, A
    movwf TARGET_X_L, A 
    movf TEMP_H, W, A
    addwfc TOTAL_ERROR_X_H, W, A
    movwf TARGET_X_H, A 
    
    ; --- DERIVATIVE GAIN X ---
    movff GYRO_X_H, TEMP_H
    movff GYRO_X_L, TEMP_L
    movf K_d, W, A
    call convert_tilt

    ; Add the gyro damping term
    movf TEMP_L, W, A
    subwf TARGET_X_L, F, A
    movf TEMP_H, W, A
    subwfb TARGET_X_H, F, A
    
    ; --- COMPARE WITH MAX/MIN VALUES
    ; Max = 0x7765, Min = 0x6765
    ;movlw 0x77
    ;cpfsgt TARGET_X_H, A
    ;movwf TARGET_X_H, A
    
    ;movlw 0x68
    ;cpfslt TARGET_X_H, A
    ;movwf TARGET_X_H, A
    
    
    ; ==============================
    ; Y-AXIS PID CALCULATIONS
    ; ==============================

    ; --- INTEGRAL GAIN Y ---
    movff ACCEL_Y_H, TEMP_H
    movff ACCEL_Y_L, TEMP_L
    movf K_i, W, A 
    call convert_tilt

    ; =========================================
    ; --- ANTI-WINDUP LOGIC ---
    ; =========================================

    ; --- 1. CHECK CEILING (Max) ---
    movlw 0x76 
    cpfslt TARGET_Y_H, A     ; If Target < 0x76, ceiling is fine, check floor
    bra check_ceiling_sign   ; Target >= 0x76! Check if we should block it
    bra check_floor          ; Target is safe from ceiling, jump to floor check

check_ceiling_sign:
    btfss TEMP_H, 7          ; Is the error positive? (Bit 7 == 0)
    bra skip_y               ; YES: It will make saturation worse. SKIP MATH!
    bra do_integration       ; NO: It will help us recover. ALLOW MATH!

    ; --- 2. CHECK FLOOR (Min) ---
check_floor:
    movlw 0x68               ; (Using 0x68 as your floor limit)
    cpfsgt TARGET_Y_H, A     ; If Target > 0x68, floor is fine, do math
    bra check_floor_sign     ; Target <= 0x68! Check if we should block it
    bra do_integration       ; Target is safe, proceed to math

check_floor_sign:
    btfsc TEMP_H, 7          ; Is the error negative? (Bit 7 == 1)
    bra skip_y               ; YES: It will make saturation worse. SKIP MATH!
    ; Fall through to do_integration

    ; =========================================
    ; --- INTEGRAL MATH ---
    ; =========================================
do_integration:
    movf TEMP_L, W, A
    addwf TOTAL_ERROR_Y_L, F, A
    movf TEMP_H, W, A
    addwfc TOTAL_ERROR_Y_H, F, A
skip_y:
    
    


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
    
    ; --- COMPARE WITH MAX/MIN VALUES
    ; Max = 0x7765, Min = 0x6765
    movlw 0x76
    cpfslt TARGET_Y_H, A
    movwf TARGET_Y_H, A
    
    movlw 0x69
    cpfsgt TARGET_Y_H, A
    movwf TARGET_Y_H, A


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