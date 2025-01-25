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
#include "uart1.h"
#include "wait.h"

#define DEBUG 1

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initHw(void)
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Initialize UART0
    initUart1();

    // Set Baud Rate = 115200
    setUart1BaudRate(9600, 40000000);
}

void move(uint32_t val)
{
    putiUart1(val);
    waitMicrosecond(1000000);
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize Hardware
    initHw();

    while(true)
    {
        move(0);
        move(10);
        move(20);
        move(30);
        move(40);
        move(50);
        move(64);
        move(74);
        move(84);
        move(94);
        move(104);
        move(114);
        move(124);
        move(0);
    }
}
