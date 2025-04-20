// Serial Library (Definition)
// File: serial.c
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

// Hardware configuration:
// UART Interface:
//   U3RX (PC6) and U3TX (PC7) are connected to the pi controller

//-----------------------------------------------------------------------------
// Device Includes
//-----------------------------------------------------------------------------

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "serial.h"
#include "wait.h"

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

// PortC masks
#define UART3_RX_MASK 64
#define UART3_TX_MASK 128

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART3
void initSerial()
{
    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R3;
    _delay_cycles(3);

    // Configure UART3 pins
    GPIO_PORTC_DIR_R |= UART3_TX_MASK;
    GPIO_PORTC_DR2R_R |= UART3_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTC_DEN_R |= UART3_TX_MASK | UART3_RX_MASK;    // enable digital on UART3 pins
    GPIO_PORTC_AFSEL_R |= UART3_TX_MASK | UART3_RX_MASK;  // use peripheral to drive PC6, PC7
    GPIO_PORTC_PCTL_R &= ~(GPIO_PCTL_PC6_M | GPIO_PCTL_PC7_M); // clear bits 0-7
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_U3RX | GPIO_PCTL_PC7_U3TX;
                                                        // select UART3 to drive pins PC6 and PC7: default, added for clarity
}

// Set baud rate as function of instruction cycle frequency
void setUart3BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc << 3) / baudRate;              // calculate divisor (r) in units of 1/128
    divisorTimes128 += 1;                                           // add 1/128 to allow rounding
    divisorTimes128 >>= 1;                                          // change units from 1/128 to 1/64
    UART3_IBRD_R = divisorTimes128 >> 6;                            // set integer value
    UART3_FBRD_R = divisorTimes128 & 0b111111;                      // set fractional value
    UART3_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;                // configure for 8N1 w/ 16-level FIFO
    UART3_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;    // turn-on UART3
}

// Non-blocking function that returns with serial data once the buffer is not empty
uint32_t getControlsSerial(uint32_t current_controls)
{
    uint32_t new_controls;
    uint8_t frontLeft;
    uint8_t frontRight;
    uint8_t backLeft;
    uint8_t backRight;

    uint32_t receive_controls_new = 0;
    static uint32_t receive_controls_previous;
    static uint8_t reliable_count;

    if (UART3_FR_R & UART_FR_RXFE) // if fifo empty
    {
        new_controls = current_controls; // return previous controls
    }
    
    else
    {
        frontLeft  = UART3_DR_R; // read first byte
        frontRight = UART3_DR_R; // read second byte
        backLeft   = UART3_DR_R; // read third byte
        backRight  = UART3_DR_R; // read fourth byte

        receive_controls_new = receive_controls_new | (frontLeft  << 24); // shift first byte to the left by 24 bits
        receive_controls_new = receive_controls_new | (frontRight << 16); // shift second byte to the left by 16 bits
        receive_controls_new = receive_controls_new | (backLeft   << 8);  // shift third byte to the left by 8 bits
        receive_controls_new = receive_controls_new | (backRight  << 0);  // shift fourth byte to the left by 0 bits

        if (receive_controls_new == receive_controls_previous)
            reliable_count++;
        else
            reliable_count = 0;

        if (reliable_count > 1)
            new_controls = receive_controls_new;
        else
            new_controls = current_controls;

        receive_controls_previous = receive_controls_new;
    }

    return new_controls;
}
