// GPIO INTERUPT  Library
// Madison Gage

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL with LCD Interface
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <src/sonar.h>
#include <stdint.h>
#include "wait.h"
#include "tm4c123gh6pm.h"

#define Trig0 (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 3*4))) //PB3
#define Trig1 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) //PA2


// PortD masks
#define ECHO0_MASK    64   // PD6
#define ECHO1_MASK    16   // PC4
// PortB mask
#define TRIG0_MASK    8    // PB3
#define TRIG1_MASK    4    // PA2

//#define PC6_MASK 64

void initSonar(){
    // Clocks
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;       // Timer 2
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // Timer 1
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
    _delay_cycles(3);

    // Configure Trig
    GPIO_PORTB_DIR_R |= TRIG0_MASK;
    GPIO_PORTB_DEN_R |= TRIG0_MASK;
    GPIO_PORTA_DIR_R |= TRIG1_MASK;
    GPIO_PORTA_DEN_R |= TRIG1_MASK;

    // Configure Timer 2
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER2_TAILR_R = 4000000;                        // set load value to 40e6 for 1 Hz interrupt rate
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    //TIMER1_IMR_R &= ~(TIMER_IMR_TATOIM);
    NVIC_EN0_R = 1 << (INT_TIMER2A-16);              // turn-on interrupt 39 (TIMER2A) in NVIC

    // Configure Timer 2
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 4000000;                        // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    //TIMER1_IMR_R &= ~(TIMER_IMR_TATOIM);
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);              // turn-on interrupt 39 (TIMER2A) in NVIC

    // Configure Echo
    GPIO_PORTD_AFSEL_R |= ECHO0_MASK;
    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD6_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD6_WT5CCP0;
    GPIO_PORTD_DEN_R |= ECHO0_MASK;

    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_BOTH;           // measure time from negative edge to negative edge
    WTIMER5_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER5_TAV_R = 0;                               // zero counter for first period
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R = 1 << (INT_WTIMER5A-16-96);          // turn-on interrupt 120 (WTIMER5A)

    // Configure Echo
    GPIO_PORTC_AFSEL_R |= ECHO1_MASK;
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC4_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_WT0CCP0;
    GPIO_PORTC_DEN_R |= ECHO1_MASK;

    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER0_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER0_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER0_CTL_R = TIMER_CTL_TAEVENT_BOTH;           // measure time from negative edge to negative edge
    WTIMER0_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER0_TAV_R = 0;                               // zero counter for first period
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN2_R = 1 << (INT_WTIMER0A-16-64);          // turn-on interrupt 110 (WTIMER0A)
}

void trig0_isr(){
    if(step0 != 2){          // the distance is so far that the bounce back was lost so we reset
        echoTime0 = 1000;
    }

    Trig0 = 1;               // setting trig high
    waitMicrosecond(10);
    Trig0 = 0;               // setting trig low
    step0 = 0;               // counter for wide timer
    // Clear Timer Interrupt
    TIMER2_ICR_R |= TIMER_ICR_TATOCINT;
}

void echo0_isr(){
    if(step0 == 0){          // we reset the wide timer count
        // Set timer 0
        WTIMER5_TAV_R = 0;
        step0 = 1;           // counter set to 1 indicating that the wide timer has been started
    } else {
        echoTime0 = WTIMER5_TAV_R/(4*58);    // when we reenter the widetimer get the time
        step0 = 2;
    }
    // Clear interrupt flag
    WTIMER5_ICR_R = TIMER_ICR_CAECINT;
}

void trig1_isr(){
    if(step1 != 2){          // the distance is so far that the bounce back was lost so we reset
        echoTime1 = 1000;
    }

    Trig1 = 1;               // setting trig high
    waitMicrosecond(10);
    Trig1 = 0;               // setting trig low
    step1 = 0;               // counter for wide timer
    // Clear Timer Interrupt
    TIMER1_ICR_R |= TIMER_ICR_TATOCINT;
}

void echo1_isr(){
    if(step1 == 0){          // we reset the wide timer count
        // Set timer 0
        WTIMER0_TAV_R = 0;
        step1 = 1;           // counter set to 1 indicating that the wide timer has been started
    } else {
        echoTime1 = WTIMER0_TAV_R/(4*58);    // when we reenter the widetimer get the time
        step1 = 2;
    }
    // Clear interrupt flag
    WTIMER0_ICR_R = TIMER_ICR_CAECINT;
}


