	#include <xc.inc>
	
global SPI_MasterInit, SPI_MasterTransmit

psect	SPI_code,class=CODE
	
SPI_MasterInit:
    
    banksel SSP2CON1
    movlw 0x32           
    movwf SSP2CON1, b
    
    banksel SSP2STAT
    bsf SSP2STAT, 6, b   ; CKE=1 ? data shifts on falling edge (Mode 3)
    
    bsf	    TRISD, 5, A ; Set SDI to input
    return
   
SPI_MasterTransmit:
    banksel SSP2BUF
    movwf   SSP2BUF, B     ; Load data using BSR
    
Wait_Transmit:
    btfss   SSP2STAT, 0, B  ; Check Buffer Full bit in correct bank
    bra     Wait_Transmit
    
    movf    SSP2BUF, W, B   ; Read received byte into W
    return

end