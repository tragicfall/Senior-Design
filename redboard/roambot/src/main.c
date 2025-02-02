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
//     GPIO PIN PD0 - Input to go Right
//     GPIO PIN PD1 - Input to go Down
//     GPIO PIN PD2 - Input to go Left
//     GPIO PIN PD3 - Input to go Up
// UART Outut Interface:
//     U1TX (PB1) and U1RX (PB0) are connected to the 1st controller
//     U2TX (PD7) and U2RX (PD6) are connected to the 2nd controller

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

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

// Controls {Up, Left, Down, Right}
#define UP         0b1000
#define LEFT       0b0100
#define DOWN       0b0010
#define RIGHT      0b0001
#define UP_LEFT    0b1100
#define UP_RIGHT   0b1001
#define DOWN_LEFT  0b0110
#define DOWN_RIGHT 0b0011

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
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize Hardware
    uint8_t controls;
    initHw();
    
    while(true)
    {
        controls = getControls();
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
            case UP_LEFT:
                moveUpLeft();
                break;
            case UP_RIGHT:
                moveUpRight();
                break;
            case DOWN_LEFT:
                moveDownLeft();
                break;
            case DOWN_RIGHT:
                moveDownRight();
                break;
            default:
                moveStop();
                break;
        }
        waitMicrosecond(4000000);
        moveStop();
        waitMicrosecond(1000000000);
    }
}
