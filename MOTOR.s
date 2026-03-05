#include <xc.inc>

global move_motor_X, move_motor_Y
    
extrn  TARGET_X_L, TARGET_X_H, TARGET_Y_L, TARGET_Y_H

 
psect motor_code, class=CODE
    
move_motor_X:
    
    movf TARGET_X_L, W, A
    cpfsgt TMR0L
    return
    
    movf TARGET_X_H, W
    cpfsgt  TMR0H ; compare w and f, skip if greater than 
    return
    
    bcf PORTD, 2, A
    return
    

move_motor_Y:
    movf TARGET_Y_L, W, A
    cpfsgt TMR0L
    return
    
    movf TARGET_Y_H, W
    cpfsgt  TMR0H ; compare w and f, skip if greater than 
    return
    
    bcf PORTD, 3, A
    return

    end