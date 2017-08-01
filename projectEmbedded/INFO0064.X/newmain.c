/* 
 * File:   newmain.c
 * Author: pierre
 *
 * Created on July 5, 2017, 12:59 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <pic.h>
#include <xc.h>

// PIC16F1455 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = ON        // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config CPUDIV = CLKDIV6 // CPU System Clock Selection Bit (CPU system clock divided by 6)
#pragma config USBLSCLK = 48MHz // USB Low SPeed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
#pragma config PLLMULT = 3x     // PLL Multipler Selection Bit (3x Output Frequency Selected)
#pragma config PLLEN = DISABLED // PLL Enable Bit (3x or 4x PLL Disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

/*
 * Test code that is suppose to blink a led at frequence of 1Hz
 */

volatile int change = 0;

void interrupt timer(void){
    if(INTCONbits.TMR0IF == 1){
        INTCONbits.TMR0IF = 0;
        TMR0 = 131;
        change = 1;
        return;
    }
    PORTCbits.RC3 = 1;
    while(1){}
}


int main() {
    int next = 0;
    OSCCON = 0b00111111; 
    OPTION_REG = 0b01000111 ;
    INTCON = 0b11111000;
    TMR0 = 130; //Timer0 set to 130 which is the offset.
    TRISC = 0b00000000;
    PORTCbits.RC2 = 1;
    //PORTCbits.RC3 = 1; why doesn't this work?
    while(1){
        if(change == 1){
            PORTCbits.RC2 = next;
            next = (++next)%2;
            change = 0;
        }
    }
    return 0;
}
