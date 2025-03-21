// Main Code
// File: main.c
// Group Members:
// - Christopher David
// - Madison Gage
// - Andrew Howard
// - Abubakar Kassim
// - Raya Sultan

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration
// GPIO Input Interface:
//     GPIO PIN PD0 - Input to go Up
//     GPIO PIN PD1 - Input to go Left
//     GPIO PIN PD2 - Input to go Down
//     GPIO PIN PD3 - Input to go Right
// UART Outut Interface:
//     U1TX (PB1) is connected to the 1st (front) controller
//     U2TX (PD7) is connected to the 2nd (back) controller
// Ultrasonic (will add a second sonar)
//     ECHO0 PIN PD6
//     TRIG0 PIN PB3 

//-----------------------------------------------------------------------------
// Device Includes
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "uart.h"
#include "wait.h"
#include "gpio.h"
#include "sonar.h"

// Temporary 
#define DEBUG_SONAR 1

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

// Controls {Up, Left, Down, Right}
#define UP         0b0001
#define LEFT       0b0010
#define DOWN       0b0100
#define RIGHT      0b1000
#define STOP       0b0000

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initHw(void)
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Initialize GPIO
    initGPIO();

    // Initialize Both UART
    initUart1();
    initUart2();

    // Set Baud Rate = 9600
    setUart1BaudRate(9600, 40000000);
    setUart2BaudRate(9600, 40000000);
}

//-----------------------------------------------------------------------------
// Interface Functions
//-----------------------------------------------------------------------------

void move(uint8_t controls)
{
    switch (controls)
    {
        case UP:
            moveUp();
            break;
        case LEFT:
            moveLeft();
            break;
        case DOWN:
            moveDown();
            break;
        case RIGHT:
            moveRight();
            break;
        default:
            moveStop();
            break;
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize Hardware
    uint8_t controls;
    initHw();
    
    while(true)
    {   
#if (DEBUG_SONAR == 1)
        if (echoTime < 800) controls = STOP;            // if less < 3ft stop moving       
        else
#endif
        controls = getControls();
        move(controls);
        waitMicrosecond(100000);
    }
}
