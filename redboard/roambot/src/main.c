//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device Includes and Defines
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "uart.h"
#include "wait.h"
#include "gpio.h"

// Wheel power levels for each direction (Front Left, Front Right, Back Left, Back Right)
#define AL_STOP          0

#define FL_REVERSE_FAST  44
#define FL_REVERSE_SLOW  54
#define FL_FULL_STOP     64
#define FL_FORWARD_SLOW  74
#define FL_FORWARD_FAST  84

#define FR_REVERSE_FAST  212
#define FR_REVERSE_SLOW  202
#define FR_FULL_STOP     192
#define FR_FORWARD_SLOW  182
#define FR_FORWARD_FAST  172

#define BL_REVERSE_FAST  44
#define BL_REVERSE_SLOW  54
#define BL_FULL_STOP     64
#define BL_FORWARD_SLOW  74
#define BL_FORWARD_FAST  84

#define BR_REVERSE_FAST  212
#define BR_REVERSE_SLOW  202
#define BR_FULL_STOP     192
#define BR_FORWARD_SLOW  182
#define BR_FORWARD_FAST  172

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

void moveUp()
{
    putiUart1(FL_FORWARD_FAST);
    putiUart2(BL_FORWARD_FAST);
    waitMicrosecond(100);
    putiUart1(FR_FORWARD_FAST);
    putiUart2(BR_FORWARD_FAST);
    waitMicrosecond(100);
}

void moveLeft()
{
    putiUart1(FL_REVERSE_FAST);
    putiUart2(BL_REVERSE_FAST);
    waitMicrosecond(100);
    putiUart1(FR_FORWARD_FAST);
    putiUart2(BR_FORWARD_FAST);
    waitMicrosecond(100);
}

void moveDown()
{
    putiUart1(FL_REVERSE_FAST);
    putiUart2(BL_REVERSE_FAST);
    waitMicrosecond(100);
    putiUart1(FR_REVERSE_FAST);
    putiUart2(BR_REVERSE_FAST);
    waitMicrosecond(100);
}

void moveRight()
{
    putiUart1(FL_FORWARD_FAST);
    putiUart2(BL_FORWARD_FAST);
    waitMicrosecond(100);
    putiUart1(FR_REVERSE_FAST);
    putiUart2(BR_REVERSE_FAST);
    waitMicrosecond(100);
}

void moveUpLeft()
{
    putiUart1(FL_FORWARD_SLOW);
    putiUart2(BL_FORWARD_SLOW);
    waitMicrosecond(100);
    putiUart1(FR_FORWARD_FAST);
    putiUart2(BR_FORWARD_FAST);
    waitMicrosecond(100);
}

void moveUpRight()
{
    putiUart1(FL_FORWARD_FAST);
    putiUart2(BL_FORWARD_FAST);
    waitMicrosecond(100);
    putiUart1(FR_FORWARD_SLOW);
    putiUart2(BR_FORWARD_SLOW);
    waitMicrosecond(100);
}

void moveDownLeft()
{
    putiUart1(FL_REVERSE_SLOW);
    putiUart2(BL_REVERSE_SLOW);
    waitMicrosecond(100);
    putiUart1(FR_REVERSE_FAST);
    putiUart2(BR_REVERSE_FAST);
    waitMicrosecond(100);
}

void moveDownRight()
{
    putiUart1(FL_REVERSE_FAST);
    putiUart2(BL_REVERSE_FAST);
    waitMicrosecond(100);
    putiUart1(FR_REVERSE_SLOW);
    putiUart2(BR_REVERSE_SLOW);
    waitMicrosecond(100);
}

void moveStop()
{
    putiUart1(AL_STOP);
    putiUart2(AL_STOP);
    waitMicrosecond(100);
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
    }
}
