	#include <xc.inc>

psect	code, abs
	
main:
	org	0x0
	goto	start

	org	0x100		    ; Main code starts here at address 0x100
start:
	movlw 0x0		    ; Hello my name is George! This is a push test!
	movwf TRISD, A		    ; Port D all inputs
	
	movlw 	0x0
	movwf	TRISJ, A	    ; Port C all outputs
	   
	movff	0x01, PORTD
	
	movlw	0x00
	movwf	0x06, A
	
	bra 	test
loop:
	movlw low highword(0x0000FF)
	movwf 0x12, A
	movlw high(0x0000FF)
	movwf 0x11, A
	movlw low(0x0000FF)
	movwf 0x10, A
	call bigDelay 
	
	movff 	0x06, PORTJ ; Move value from register 6 to Port C
	
	movlw	0x00
	movwf	PORTD, A
	
	movlw	0x01
	movwf	PORTD, A
	
	incf 	0x06, W, A ; Increment value of register 6, and store in W
test:   
	movwf	0x06, A	    ; Test for end of loop condition
	movlw	0xFF
	cpfsgt 	0x06, A	    ; Compares register 6 to what is stored in W
	bra 	loop		    ; Not yet finished goto start of loop again
	
	goto 	0x0		    ; Re-run program from start
	
bigDelay:
    movlw 0x00
DLoop:
    decf 0x10, f, A
    subwfb 0x11, f, A
    subwfb 0x12, f, A
    bc DLoop
    return

	end	main
