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

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initHw(void)
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Initialize Both UART
    initUart1();
    initUart2();

    // Set Baud Rate = 9600
    setUart1BaudRate(9600, 40000000);
    setUart2BaudRate(9600, 40000000);
}

void moveForward()
{
    putiUart1(84);
    putiUart2(84);
    waitMicrosecond(100);
    putiUart1(172);
    putiUart2(172);
    waitMicrosecond(100);
}

void moveStop()
{
    putiUart1(0);
    putiUart2(0);
    waitMicrosecond(100);
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
        moveForward();
        waitMicrosecond(4000000);
        moveStop();
        waitMicrosecond(4000000);
    }
}
