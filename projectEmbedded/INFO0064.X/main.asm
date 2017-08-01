; File lid a led.
; Assembly code for pic16f1455

; Turn on a led connected to pin RC2
; Use int osc about 16Mhz

;CPU config
;   PIC16F1455 (internal oscilator, watch-dog off, Power-up timer on)
    
    #include <pic16f1455.inc>
    __CONFIG _FOSC_INTOSC _WDTE_OFF _PWRTE_ON _MCLRE_ON _CP_OFF _BOREN_ON _CLKOUTEN_OFF _IESO_ON _FCMEN_ON 
; Program   
 
    org 0		;program origine
    bcf TRISC,2
    
    movlw B'00000100'	;third bit to 1
    movwf PORTC
    
fin: goto fin
    end