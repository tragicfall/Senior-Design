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
// UART Input Interface:
//     U3RX (PC6) is connected to the pi controller
// UART Outut Interface:
//     U1TX (PB1) is connected to the 1st (front) controller
//     U2TX (PD7) is connected to the 2nd (back) controller
// Ultrasonic (will add a second sonar)
//     ECHO0 PIN PD6
//     TRIG0 PIN PB3
//     ECHO1 PIN PC4
//     TRIG1 PIN PA2

//-----------------------------------------------------------------------------
// Device Includes
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "uart.h"
#include "wait.h"
#include "serial.h"
#include "src/sonar.h"

// Temporary 
#define DEBUG_SONAR 0
#define NAV         1   // difference between a human and the lidar?

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

    // Initialize Serial
    initSerial();

    // Initialize Both UART
    initUart1();
    initUart2();

    // Set Baud Rate = 9600
    setUart1BaudRate(9600,   40000000);
    setUart2BaudRate(9600,   40000000);
    setUart3BaudRate(115200, 40000000);
}

//-----------------------------------------------------------------------------
// Interface Functions
//-----------------------------------------------------------------------------

void move(uint32_t controls)
{
    uint8_t frontLeft;
    uint8_t frontRight;
    uint8_t backLeft;
    uint8_t backRight;

    frontLeft  = (controls >> 24) & 0xFF;
    frontRight = (controls >> 16) & 0xFF;
    backLeft   = (controls >> 8) & 0xFF;
    backRight  = (controls >> 0) & 0xFF;

    moveWheels(frontLeft, frontRight, backLeft, backRight);
}

void caution(uint32_t controls)
{
    static uint8_t stop_l = 0,
                   stop_r = 0;
    switch (controls)
    {
        case UP:
            moveDown();             // Go backwards
            waitMicrosecond(3000000); // Allow the lidar time to assess
            stop_l = 0;
            stop_r = 0;
            break;
        case LEFT:
            if(stop_l == 3)         // If stopped 3 times during turning we back up a bit and start again
            {
                stop_l = 0;
                moveDown();
                waitMicrosecond(3000000);
            }
            stop_l++;
            stop_r = 0;
            break;
        case DOWN:
            stop_l = 0;
            stop_r = 0;
            break;
        case RIGHT:
            if(stop_r == 3)
            {
                stop_r = 0;
                moveDown();
                waitMicrosecond(3000000);
            }
            stop_r++;
            stop_l = 0;
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
    uint32_t controls;
    initHw();
    initSonar();
    
    while(true)
    {   
#if (DEBUG_SONAR == 1)
        if ((echoTime0 < 800) || (echoTime1 < 800)) {
            moveStop();
            waitMicrosecond(3000000);
            caution(controls);
        }
#endif
        controls = getControlsSerial(controls);
        move(controls);
        waitMicrosecond(1000);
    }
}


/*
 * Controls go as normal until 800 detected. needs to be able to get itself out of trouble
 * so reverse
 */
