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

#define Trig (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 3*4))) //PB3

// PortD masks
#define ECHO_MASK    64   // PD6
// PortB mask
#define TRIG_MASK    8    // PB3

//#define PC6_MASK 64

void initSonar(){
    // Clocks
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;       // Timer 2
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
    _delay_cycles(3);

    // Configure Trig
    GPIO_PORTB_DIR_R |= TRIG_MASK;
    GPIO_PORTB_DEN_R |= TRIG_MASK;

    // Configure Timer 2
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER2_TAILR_R = 4000000;                        // set load value to 40e6 for 1 Hz interrupt rate
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    //TIMER1_IMR_R &= ~(TIMER_IMR_TATOIM);
    NVIC_EN0_R = 1 << (INT_TIMER2A-16);              // turn-on interrupt 39 (TIMER2A) in NVIC

    // Configure Echo
    GPIO_PORTD_AFSEL_R |= ECHO_MASK;
    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD6_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD6_WT5CCP0;
    GPIO_PORTD_DEN_R |= ECHO_MASK;

    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_BOTH;           // measure time from negative edge to negative edge
    WTIMER5_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER5_TAV_R = 0;                               // zero counter for first period
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R = 1 << (INT_WTIMER5A-16-96);          // turn-on interrupt 120 (WTIMER5A)
}

void time2_isr(){
    if(step != 2){
        echoTime = -1;
    }

    Trig = 1;
    waitMicrosecond(10);
    Trig = 0;
    step = 0;
    // Clear Timer Interrupt
    TIMER2_ICR_R |= TIMER_ICR_TATOCINT;
}

void echo_isr(){
    if(step == 0){
        // Set timer 0
        WTIMER5_TAV_R = 0;
        step = 1;
    } else {
        echoTime = WTIMER5_TAV_R/(4*58);
        step = 2;
    }
    // Clear interrupt flag
    WTIMER5_ICR_R = TIMER_ICR_CAECINT;
}




