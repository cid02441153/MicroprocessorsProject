#include <xc.inc>
    
global stabilise_motors, initialise_PID
global TARGET_X_L, TARGET_X_H, TARGET_Y_L, TARGET_Y_H
global FILTERED_X_H, FILTERED_Y_H
global TARGET_X_H, TARGET_Y_H

extrn GYRO_X_L, GYRO_X_H, GYRO_Y_L, GYRO_Y_H
extrn ACCEL_X_H, ACCEL_X_L, ACCEL_Y_H, ACCEL_Y_L

psect udata_acs 
SHIFT_H: ds 1
SHIFT_L: ds 1
    
; Variables for PWM signal to compare to timer0
TARGET_X_L: ds 1
TARGET_X_H: ds 1
TARGET_Y_L: ds 1
TARGET_Y_H: ds 1
    
; Total error (cumulative distance from 0) for integral gain
TOTAL_ERROR_X_L: ds 1 
TOTAL_ERROR_X_H: ds 1     
TOTAL_ERROR_Y_L: ds 1 
TOTAL_ERROR_Y_H: ds 1 
    
FILTERED_X_H: ds 1
FILTERED_X_L: ds 1
FILTERED_Y_L: ds 1
FILTERED_Y_H: ds 1
    
; Temp variables so that routines can be used for both X and Y
TEMP_H: ds 1
TEMP_L: ds 1
   
; PID values
K_p_x: ds 1
K_i_x: ds 1
K_d_x: ds 1
K_p_y: ds 1
K_i_y: ds 1
K_d_y: ds 1
    
counter: ds 1
    
psect pid_code, class=CODE
initialise_PID:
    ; --- INITIALISE PID VALUES ---
    movlw 0x04
    movwf K_p_x, A
    movlw 0x03
    movwf K_i_x, A
    movlw 0x07
    movwf K_d_x, A
    
    movlw 0x03
    movwf K_p_y, A
    movlw 0x01
    movwf K_i_y, A
    movlw 0x08
    movwf K_d_y, A
    
    ; --- INITIALISE DEFAULT POSITION ---
    movlw 0x6F
    movwf TARGET_X_H, A
    movlw 0x65
    movwf TARGET_X_L, A
    
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
    
    return
    
stabilise_motors:
    call convert_tilt_X
    call convert_tilt_Y
    return
    
convert_tilt_X:
    
    ; --- PROPORTIONAL GAIN X ---
    
    ; we want to do 98% of gyroscope + 2% of accelerometer
    movff GYRO_X_L, SHIFT_L
    movff GYRO_X_H, SHIFT_H
    movlw 0x04
    call right_shift_W ; Divide GYRO by 16
    
    movf SHIFT_L, W
    addwf FILTERED_X_L, F, A
    
    movf SHIFT_H, W
    addwfc FILTERED_X_H, F, A
    
    ; Subtract accel from filtered
    movf FILTERED_X_L, W, A
    subwf ACCEL_X_L, W, A
    movwf SHIFT_L, A
    
    movf FILTERED_X_H, W, A
    subwfb ACCEL_X_H, W, A
    movwf SHIFT_H, A
    
    movlw 0x06
    call right_shift_W
    
    movf FILTERED_X_L, W, A
    addwf SHIFT_L, W, A
    movwf FILTERED_X_L
    
    movf FILTERED_X_H, W, A
    addwfc SHIFT_H, W, A
    movwf FILTERED_X_H
    
    movff FILTERED_X_H, TEMP_H
    movff FILTERED_X_L, TEMP_L
    movf K_p_x, W, A 
    call convert_tilt

    ; Subtract the proportional term from the total error
    movf TEMP_L, W, A
    subwf TOTAL_ERROR_X_L, W, A
    movwf TARGET_X_L, A 
    movf TEMP_H, W, A
    subwfb TOTAL_ERROR_X_H, W, A
    movwf TARGET_X_H, A 
    
    ; --- INTEGRAL GAIN X ---
    movff ACCEL_X_H, TEMP_H
    movff ACCEL_X_L, TEMP_L
    movf K_i_x, W, A 
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
    
    ; --- DERIVATIVE GAIN X ---
    movff GYRO_X_H, TEMP_H
    movff GYRO_X_L, TEMP_L
    movf K_d_x, W, A
    call convert_tilt

    ; Add the gyro damping term
    movf TEMP_L, W, A
    subwf TARGET_X_L, F, A
    movf TEMP_H, W, A
    subwfb TARGET_X_H, F, A
    
    return
    
    
convert_tilt_Y:
    ; we want to do 98% of gyroscope + 2% of accelerometer
    movff GYRO_Y_L, SHIFT_L
    movff GYRO_Y_H, SHIFT_H
    movlw 0x04
    call right_shift_W ; Divide GYRO by 16
    
    movf SHIFT_L, W
    addwf FILTERED_Y_L, F, A
    
    movf SHIFT_H, W
    addwfc FILTERED_Y_H, F, A
    
    ; Subtract accel from filtered
    movf FILTERED_Y_L, W, A
    subwf ACCEL_Y_L, W, A
    movwf SHIFT_L, A
    
    movf FILTERED_Y_H, W, A
    subwfb ACCEL_Y_H, W, A
    movwf SHIFT_H, A
    
    movlw 0x06
    call right_shift_W
    
    movf FILTERED_Y_L, W, A
    addwf SHIFT_L, W, A
    movwf FILTERED_Y_L
    
    movf FILTERED_Y_H, W, A
    addwfc SHIFT_H, W, A
    movwf FILTERED_Y_H
    
    movff FILTERED_Y_H, TEMP_H
    movff FILTERED_Y_L, TEMP_L
    movf K_p_y, W, A
    call convert_tilt

    ; Subtract the proportional term from the total error
    movf TEMP_L, W, A
    addwf TOTAL_ERROR_Y_L, W, A
    movwf TARGET_Y_L, A 
    movf TEMP_H, W, A
    addwfc TOTAL_ERROR_Y_H, W, A
    movwf TARGET_Y_H, A 
    
    ; --- INTEGRAL GAIN Y ---
    movff ACCEL_Y_H, TEMP_H
    movff ACCEL_Y_L, TEMP_L
    
    movf K_i_y, W, A 
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
    
    ; --- DERIVATIVE GAIN Y ---
    movff GYRO_Y_H, TEMP_H
    movff GYRO_Y_L, TEMP_L
    movf K_d_y, W, A
    call convert_tilt

    ; Add the gyro damping term
    movf TEMP_L, W, A
    addwf TARGET_Y_L, F, A
    movf TEMP_H, W, A
    addwfc TARGET_Y_H, F, A
    
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
    
