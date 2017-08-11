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

volatile char* msg = NULL; //message to be sent 
volatile uint8_t i = 0; //varibale to iterrate over the msg
volatile uint8_t done = 0; //1 when message has been send
volatile uint8_t change = 1;
volatile uint16_t conversion=0; //Value from a ADC
volatile uint16_t sharpInter[3] = 0;
volatile uint8_t j = 0;
volatile uint16_t batteryInter = 0;
volatile uint8_t uartValueInter = 0; // value read on the RX pin
volatile uint8_t TIMER0_START = 0; // initial value for timer0 register

volatile uint8_t bt = 0;

void ADC_init(void){
    //RC0 is used for the sharp sensor
    TRISCbits.TRISC0 = 0b1; // Pin RC0 is an input pin
    ANSELCbits.ANSC0 = 0b1; // Pin RC0 is an analog Pin
    //RC1 is used to detect battery energy
    TRISCbits.TRISC1 = 0b1; // Pin RC1 is an input pin
    ANSELCbits.ANSC1 = 0b1; // Pin RC1 is an analog Pin
    
    /*Right justified. Six Most Significant bits of ADRESH
     *  are set to ?0? when the conversion result is loaded*/
    ADCON1bits.ADFM = 0b1; 
    ADCON1bits.ADCS = 0b100; // ADC clock select bits FOSC/4
    ADCON1bits.ADPREF = 0b00; //Vref+ is connected to VDD.
    
    ADCON0bits.CHS = 0b00100; // Channel AN4 is selected
    
    ADCON2bits.TRIGSEL = 0b000; // No auto conversion.
    
    ADCON0bits.ADON = 0b1; //ADC is enable
    return;
}

void UART_init(void){
    BAUDCONbits.BRG16 = 0b1; //baud rate generator is a 16bit timer
    TXSTAbits.BRGH = 0b1;
    SPBRGH = 0;
    SPBRGL = 0b00110100; //Free running period of baud rate generator, found by
    //solving equation of page 262 of controller datasheet for a baud rate of 9600  
    TRISCbits.TRISC5 = 0b1; //RX input pin
    TRISCbits.TRISC4 = 0b0; //TX output pin
    RCSTAbits.CREN = 0b1; //enable the receiver circuit
    
    TXSTAbits.SYNC = 0b0; //configured for asynchronous operations
    TXSTAbits.TXEN = 0b1; //Enables the transmitter circuit
    RCSTAbits.SPEN = 0b1; //enable UART
    RCSTAbits.RX9 = 0b0; //no ninth bits
    TXSTAbits.TX9 = 0b0;
    return;
}

void PWM_init(void){
    APFCONbits.P2SEL = 0b0; //PWM2 on pin RC3
    TRISCbits.TRISC3 = 0b0; //RC3 output mode
    PWM2CON = 0; //DISABLE PWM
    PR2 = 155; //20ms at 2MHz TOSC
    PWM2DCH = 0b00000110; //50% of period
    PWM2DCL = 0b00; //Drop the 2 last bits of precision
    PIR1bits.TMR2IF = 0b0; //clearing TMR2 interrupt flag
    T2CONbits.T2CKPS = 0b11; //PRESCALER 64
    T2CONbits.TMR2ON = 0b1; //TMR2 on 
    PWM2CON = 0b11000000; // PWM enable and output enable
}

void TIMER0_init(void){
    OPTION_REGbits.nWPUEN = 0b1; // weak pull-ups are disable
    OPTION_REGbits.INTEDG = 0b1; // interrupt on rising edge
    OPTION_REGbits.TMR0CS = 0b0; // Transition on FOSC/4
    OPTION_REGbits.TMR0SE = 0b0; // Transition on low to high
    OPTION_REGbits.PSA = 0b0; // Prescaler assigned to Timer
    OPTION_REGbits.PS = 0b111; // 1:256 prescaler value
    //250 ms
    TIMER0_START = 12;
}

void TIMER1_init(void){
    T1CONbits.TMR1CS = 0b00; // FOSC/4
    T1CONbits.T1CKPS = 0b11; // 1:8 prescaler
    T1CONbits.T1OSCEN = 0b0; //Dedicated Timer1 oscillator circuit disabled
    T1CONbits.nT1SYNC = 0b1; // Do not synchronize asynchronous clock input
    PIE1bits.TMR1IE = 0b1;
    T1CONbits.TMR1ON = 0b1;
}

int calibrate(){
    while(1){
        //wait for next button to be clicked
        if(uartValueInter == 0x31){
            if(sharpInter[2]){
                INTCONbits.GIE = 0b0;
                uartValueInter = 0;
                INTCONbits.GIE = 0b1;
                return sharpInter[2];
            }
        }
    }
    return 0;
}

void send(char* str){
    msg = str;
    i = 0;
    PIE1bits.TXIE = 0b1;
    while(!done){}
}

void interrupt interrupt_service(void){
    /*for reception of the message we must ensure that we have activated the
     * TXIE bit as TXIF flag is set even when TXIE is cleared    
    */
    if(PIE1bits.TXIE){
        if(PIR1bits.TXIF){
            if(msg != NULL && msg[i] != '\0'){           
                TXREG = msg[i];
                ++i;
            }else{
                msg = NULL;
                PIE1bits.TXIE = 0b0; //disable sending
                done = 1;
            }
        }
        return;
    }
    if(INTCONbits.TMR0IF){ // timer interrupt
        INTCONbits.TMR0IF = 0b0;
        TMR0 = TIMER0_START;
        if(change == 2 || change == 0)
            ADCON0bits.GO_nDONE = 0b1;
        change = (++change)%4;
        return;
    }
    if(PIR1bits.TMR1IF){
        PIR1bits.TMR1IF = 0b0;
        TMR1H = 0;
        TMR1L = 0;
        bt = 0;
    }
    if(PIR1bits.ADIF){ // A/D interrupt
        conversion = ((uint16_t)ADRESH << 8) | ADRESL;
        PIR1bits.ADIF = 0;
        
        if(change == 3){
            //we just got info from the sensor we switch to the battery
            sharpInter[j] = conversion;
            if(sharpInter[j] >= sharpInter[(j+1)%2] - 10 || sharpInter[j] <= sharpInter[(j+1)%2] + 10){
                sharpInter[2] = conversion;
                sharpInter[0] = 0;
                sharpInter[1] = 0;
            }else{
                j = (j+1)%2;
            }
            ADCON0bits.CHS = 0b00101;
        }else if(change == 1){
            //inversely we listen to the sharp sensor
            batteryInter = conversion;
            ADCON0bits.CHS = 0b00100;
        }
        conversion = 0;
        return;
    }
    if(PIR1bits.RCIF){ //UART interrupt
        if(RCSTAbits.FERR){
            uartValueInter = RCREG;
            uartValueInter = 0;
            return;
        }
        uartValueInter = RCREG;
        return;
    }
    return;
}

int main() {
    //State variables
    
    uint16_t battery = 0;
    uint8_t uartValue = 0;
    uint16_t sharp = 0;
    
    uint8_t sensor = 0;
    uint8_t door = 0;
    //Duty cycle value for the servo
    const uint8_t OPEN = 0b00000100;
    const uint8_t CLOSE = 0b00010001;
    //Distance for the different states
    uint16_t HAND = 312;
    uint16_t NO_HAND = 200;
    uint16_t DOOR_OPEN = 100;
    
    uint16_t LOW_BATTERY = 800;
    
    OSCCONbits.IRCF = 0b1100; //clock at 2MHz
    OSCCONbits.SPLLEN = 0b0; //PLL disable
    OSCCONbits.SCS = 0b11; //Internal clock
        
    INTCON = 0b11100000; //1 global interrupt, 1 Peripheral interrupts,1 timer0 
    //overflow interrupt, 1 external interrupt,1 Interrupt-on-Change Enable, 0 timer0 flag
    //0 external flag, 0 non interrupt on change
    
    PIE1 = 0b01110000; //enable interrupt for A/D and UART
    
    TMR0 = 0;
    TRISAbits.TRISA5 = 0b0;
    TRISCbits.TRISC2 = 0b0;
    TIMER0_init();
    ADC_init();
    UART_init();
    PWM_init();
    
    while(1){
        
        //check sharp
        
        if(sharpInter[2]){
            INTCONbits.GIE = 0b0;
            sharp = sharpInter[2];
            sharpInter[2] = 0;
            INTCONbits.GIE = 0b1;
            if(sharp >= HAND){ // a hand is visible
                sensor = 1;
                door = 0;
                //LATCbits.LATC2 = 0b1;
                //LATAbits.LATA5 = 0b0;
            }else if(sharp <= DOOR_OPEN){ // door is opened
                sensor = 0;
                door = 1;
                //LATCbits.LATC2 = 0b0;
                //LATAbits.LATA5 = 0b1;
            }else{ // door closed and no hand
                sensor = 0;
                door = 0;
                //LATCbits.LATC2 = 0b1;
                //LATAbits.LATA5 = 0b1;
            }
        }
        
        //check battery
       
        if(batteryInter){
            INTCONbits.GIE = 0b0;
            battery = batteryInter;
            batteryInter = 0;
            INTCONbits.GIE = 0b1;
            if(battery <= LOW_BATTERY){
                LATCbits.LATC2 = 0b1;
                LATAbits.LATA5 = 0b0; 
            }else{
                LATCbits.LATC2 = 0b0;
                LATAbits.LATA5 = 0b1;
            }
        }
        
        
        //check if uart
        if(uartValueInter){
            INTCONbits.GIE = 0b0;
            uartValue = uartValueInter;
            uartValueInter = 0;
            INTCONbits.GIE = 0b1;
            if(uartValue == 0x31){
                bt = 1;
                LATCbits.LATC1 = 0b1;
            }else if(uartValue == 0x32){   
                bt = 0;
                LATCbits.LATC1 = 0b0;
            }else if(uartValue == 0x33){
                PWM2DCH = OPEN;
                send("hand\r\n");
                HAND = calibrate();
                uartValue = 0;
                send("noHand\r\n");
                NO_HAND = calibrate();
                DOOR_OPEN = abs(NO_HAND - 100);
                send("OK\r\n");
            }
            uartValue = 0;
        }
        
        if(sensor || bt || door){
            PWM2DCH = OPEN;
        }else{
            PWM2DCH = CLOSE;
        }
    }
    return 0;
}