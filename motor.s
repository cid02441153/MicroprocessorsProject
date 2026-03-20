#include <xc.inc>

global move_motor_X, move_motor_Y, init_angle
    
extrn  TARGET_X_L, TARGET_X_H, TARGET_Y_L, TARGET_Y_H

 
psect motor_code, class=CODE
    
move_motor_X:
    
    movlw 0x77
    cpfslt TARGET_X_H, A 
    bra clamp_max_X
    
    movlw 0x68
    cpfsgt TARGET_X_H, A 
    bra clamp_min_X
    
    bra clamp_done_X
    
clamp_max_X:
    movlw 0x77
    movwf TARGET_X_H, A
    movlw 0x37
    movwf TARGET_X_L, A
    bra clamp_done_X
    
clamp_min_X:
    movlw 0x68
    movwf TARGET_X_H, A
    movlw 0x00
    movwf TARGET_X_L, A
    bra clamp_done_X
    
clamp_done_X:    
    
    movf TARGET_X_L, W, A
    subwf TMR0L, W, A
    
    movf TARGET_X_H, W, A
    subwfb TMR0H, W, A
    
    bc turn_off_X 
    
    return
    
turn_off_X:
    bcf PORTD, 2, A
    return
    

move_motor_Y:
    
    movlw 0x77 
    cpfslt TARGET_Y_H, A 
    bra clamp_max_Y
    
    movlw 0x68 
    cpfsgt TARGET_Y_H, A 
    bra clamp_min_Y
    
    bra clamp_done_Y
    
clamp_max_Y:
    movlw 0x77
    movwf TARGET_Y_H, A
    movlw 0x37
    movwf TARGET_Y_L, A
    bra clamp_done_Y
    
clamp_min_Y:
    movlw 0x68
    movwf TARGET_Y_H, A
    movlw 0x00
    movwf TARGET_Y_L, A
    bra clamp_done_Y
    
clamp_done_Y:    
    
    movf TARGET_Y_L, W, A
    subwf TMR0L, W, A
    
    movf TARGET_Y_H, W, A
    subwfb TMR0H, W, A
    
    bc turn_off_Y 
    
    return
    
turn_off_Y:
    bcf PORTD, 3, A
    return

   
; Set an initial angle for the servo for testing
init_angle:
    ; Max angle = 0x7700
    ; Min angle = 0x6800
    movlw 0x68
    movwf TARGET_X_H, A
    
    movlw 0x00
    movwf TARGET_X_L, A
   
    movlw 0x68
    movwf TARGET_Y_H, A
   
    movlw 0x00
    movwf TARGET_Y_L, A
    
    return
    
    end