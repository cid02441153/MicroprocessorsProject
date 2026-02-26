#include <xc.inc>

global Setup_Accel, Read_Accel
extrn  ACCEL_X_L, ACCEL_X_H, ACCEL_Y_L, ACCEL_Y_H

psect accel_code, class=CODE

Setup_Accel:
    ; Configure SPI pins on Port D
    banksel TRISD
    bcf TRISD, 0, b     ; RD0 = CS output
    bcf TRISD, 6, b     ; RD6 = SCK output
    bcf TRISD, 4, b     ; RD4 = SDO output
    bsf TRISD, 5, b     ; RD5 = SDI input

    ; CS starts high
    banksel LATD
    bsf LATD, 0, b

    ; Init SPI - matches working code exactly
    banksel SSP2CON1
    movlw 0x32
    movwf SSP2CON1, b

    call Wake_Accel
    call Enable_Increment
    return

Wake_Accel:
    banksel LATD
    bcf LATD, 0, b      ; CS low

    movlw 0x20
    call SPI_Xchg       ; CTRL_REG6_XL
    movlw 0x60
    call SPI_Xchg       ; 119Hz, +-2g

    banksel LATD
    bsf LATD, 0, b      ; CS high
    return

Enable_Increment:
    banksel LATD
    bcf LATD, 0, b      ; CS low

    movlw 0x22
    call SPI_Xchg       ; CTRL_REG8
    movlw 0x44
    call SPI_Xchg       ; IF_ADD_INC=1, BDU=1

    banksel LATD
    bsf LATD, 0, b      ; CS high
    return

Read_Accel:
    banksel LATD
    bcf LATD, 0, b      ; CS low

    movlw 0xA8          ; read | auto-increment | reg 0x28
    call SPI_Xchg

    movlw 0x00
    call SPI_Xchg
    movwf ACCEL_X_L, c

    movlw 0x00
    call SPI_Xchg
    movwf ACCEL_X_H, c

    movlw 0x00
    call SPI_Xchg
    movwf ACCEL_Y_L, c

    movlw 0x00
    call SPI_Xchg
    movwf ACCEL_Y_H, c

    banksel LATD
    bsf LATD, 0, b      ; CS high
    return

; SPI exchange - copied exactly from working code
SPI_Xchg:
    movwf SSP2BUF, c
WS: btfss SSP2STAT, 0, c
    bra WS
    movf SSP2BUF, W, c
    return

    end