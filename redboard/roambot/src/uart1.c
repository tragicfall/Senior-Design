// UART1 Library
// Andrew Howard

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U0TX (PB1) and U0RX (PB0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart1.h"

// PortB masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART1
void initUart1()
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    _delay_cycles(3);

    // Configure UART1 pins
    GPIO_PORTB_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART1 pins
    GPIO_PORTB_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PB0, PB1
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_M | GPIO_PCTL_PB0_M); // clear bits 0-7
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX;
                                                        // select UART1 to drive pins PB0 and PB1: default, added for clarity

    // Configure UART1 to 115200
    UART1_CTL_R = 0;                                    // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART1_IBRD_R = 21;                                  // r = 40 MHz / (Nx19.2kHz), set floor(r)=130, where N=16
    UART1_FBRD_R = 45;                                  // round(fract(r)*64)=13
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // enable TX, RX, and module
}

// Set baud rate as function of instruction cycle frequency
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc << 3) / baudRate;              // calculate divisor (r) in units of 1/128
    divisorTimes128 += 1;                                           // add 1/128 to allow rounding
    divisorTimes128 >>= 1;                                          // change units from 1/128 to 1/64
    UART1_IBRD_R = divisorTimes128 >> 6;                            // set integer value
    UART1_FBRD_R = divisorTimes128 & 0b111111;                      // set fractional value
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;                // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;    // turn-on UART0
}

void putiUart1(uint32_t i)
{
    while (UART1_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART1_DR_R = i;                                  // write character to fifo
}
