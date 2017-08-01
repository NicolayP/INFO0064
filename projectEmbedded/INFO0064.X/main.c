/* 
 * File:   newmain.c
 * Author: pierre
 *
 * Created on July 5, 2017, 12:59 PM
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <pic.h>
#include <xc.h>

// PIC16F1455 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF        // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config CPUDIV = CLKDIV6 // CPU System Clock Selection Bit (CPU system clock divided by 6)
#pragma config USBLSCLK = 48MHz // USB Low SPeed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
#pragma config PLLMULT = 3x     // PLL Multipler Selection Bit (3x Output Frequency Selected)
#pragma config PLLEN = DISABLED // PLL Enable Bit (3x or 4x PLL Disabled)
#pragma config STVREN = OFF      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

/*
 * Test code that is suppose to blink a led at frequence of 1Hz
 */

volatile uint8_t change = 0;
volatile uint16_t converstion=0;
volatile uint8_t uartValue = 0;

void ADC_init(void){
    ANSELCbits.ANSC0 = 0b1;
    ADCON1bits.ADFM = 0b1;
    ADCON1bits.ADCS = 0b010;
    ADCON1bits.ADPREF = 0b00;
    ADCON0bits.CHS = 0b00100;
    ADCON2bits.TRIGSEL = 0b011;
    ADCON0bits.ADON = 0b1;
    return;
}

void UART_init(void){
    BAUDCONbits.BRG16 = 0b1; //baud rate generator is a 16bit timer
    TXSTAbits.BRGH = 0b1;
    SPBRGH = 0;
    SPBRGL = 0b00110100; //Free running period of baud rate generator, found by
    //solving equation of page 262 of controller datasheet for a baud rate of 38400
    TRISCbits.TRISC5 = 0b1;
    RCSTAbits.CREN = 0b1; //enable the receiver circuit
    TXSTAbits.SYNC = 0b0; //configured for asynchronous operations
    RCSTAbits.SPEN = 0b1; //enable UART
    RCSTAbits.RX9 = 0b0; //no ninth bits
    
    return;
}

void PWM_init(void){
    APFCONbits.P2SEL = 0b0; //PWM2 on pin RC3
    TRISCbits.TRISC3 = 0b0; //RC3 output mode
    PWM2CON = 0; //DISABLE PWM
    PR2 = 155; //20ms at 2MHz TOSC
    PWM2DCH = 0b00000110; //50% of period
    PWM2DCL = 0b00; //Drop the 2 last bits of precisionx²
    PIR1bits.TMR2IF = 0b0; //clearing TMR2 interrupt flag
    T2CONbits.T2CKPS = 0b11; //PRESCALER 64
    T2CONbits.TMR2ON = 0b1; //TMR2 on 
    PWM2CON = 0b11000000; // PWM enable and output enable
}


void interrupt timer(void){
    if(INTCONbits.TMR0IF){ // timer interrupt
        INTCONbits.TMR0IF = 0;
        TMR0 = 0;
        change = 0;
        return;
    }
    if(PIR1bits.ADIF){ // A/D interrupt
        converstion = ((uint16_t)ADRESH << 8) | ADRESL;
        PIR1bits.ADIF = 0;
        return;
    }
    if(PIR1bits.RCIF){ //UART interrupt
        if(RCSTAbits.FERR){
            uartValue = RCREG;
            uartValue = 0;
            return;
        }
        uartValue = RCREG;
        return;
    }
    return;
}

int main() {
    OSCCONbits.IRCF = 0b1100; //clock at 2MHz
    OSCCONbits.SPLLEN = 0b0; //PLL disable
    OSCCONbits.SCS = 0b11; //Internal clock
    
    OPTION_REG = 0b01000111;
    
    INTCON = 0b11100000; //1 global interrupt, 1 Peripheral interrupts,1 timer0 
    //overflow interrupt, 1 external interrupt,1 Interrupt-on-Change Enable, 0 timer0 flag
    //0 external flag, 0 non interrupt on change
    
    PIE1 = 0b01110000; //enable interrupt for A/D and UART
    
    TMR0 = 0;
    TRISC = 0b00100001;
    
    ADC_init();
    UART_init();
    PWM_init();
    while(1){
        if(converstion){
            if(converstion > 312){
                PWM2DCH = 0b00010001; 
                LATCbits.LATC2 = 0b1;
            }else{
                PWM2DCH = 0b00000100;
                LATCbits.LATC2 = 0b0;
            }
            converstion = 0;
        } 
        if(uartValue){
            if(uartValue == 0x31){
                LATCbits.LATC1 = 0b1;
                PWM2DCH = 0b00000100;
            }else if(uartValue == 0x32){   
                LATCbits.LATC1 = 0b0;
                PWM2DCH = 0b00010001;
            }else{
                LATCbits.LATC2 = 0b1;
            }
            uartValue = 0;
        }
    }
    return 0;
}