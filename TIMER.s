#include <xc.inc>
	
global	DAC_Setup, DAC_Int_Hi
extrn	rotate
    
psect	dac_code, class=CODE
	
DAC_Int_Hi:	
	btfss	TMR0IF		; check that this is timer0 interrupt
	retfie	f		; if not then return
	incf	LATJ, F, A	; increment PORTD
	
	; Set both ports D2 and D3 high (for both motors)
	bsf	LATD, 2, A
	bsf	LATD, 3, A
	
	bcf	TMR0IF		; clear interrupt flag
	
	; Give the timer an initial start time
	call timer_reset
	
	call rotate
	
	retfie	f		; fast return from interrupt
	
DAC_Setup:
	movlw	10000010B	; Set timer0 to 16-bit, Fosc/4/256
	movwf	T0CON, A	; = 62.5KHz clock rate, approx 1sec rollover
	bsf	TMR0IE		; Enable timer0 interrupt
	bsf	GIE		; Enable all interrupts
	
	call timer_reset
	return
	
timer_reset: ; Set the timer to a given number rather than 0
    ; Value of 0x63AC -> 0xFFFF -> 20ms cycle
    movlw 0x63
    movwf TMR0H, A
    
    movlw 0xAC
    movwf TMR0L, A
    
    return
	
	end