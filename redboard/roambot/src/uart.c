// UART Library (Definition)
// File: uart.c
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
//   U1TX (PB1) and U1RX (PB0) are connected to the 1st controller
//   U2TX (PD7) and U2RX (PD6) are connected to the 2nd controller

//-----------------------------------------------------------------------------
// Device Includes
//-----------------------------------------------------------------------------

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "uart.h"
#include "wait.h"

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

// PortB masks
#define UART1_TX_MASK 2
#define UART1_RX_MASK 1

// PortD masks
#define UART2_RX_MASK 64
#define UART2_TX_MASK 128

// Wheel Power Levels (Front Left, Front Right, Back Left, Back Right)

// Move Up: front two wheels are slow (rover is back heavy)
#define U_FL          9
#define U_FR          9
#define U_BL          8
#define U_BR          8

// Move Left: back right wheel is slow
#define L_FL          20
#define L_FR          20
#define L_BL          20
#define L_BR          24

// Move Down: back two wheels are fast (rover is back heavy)
#define D_FL          8
#define D_FR          8
#define D_BL          8
#define D_BR          8

// Move Right: power tuning seems ok
#define R_FL          20
#define R_FR          20
#define R_BL          20
#define R_BR          20

#define STOP          0

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
    GPIO_PORTD_DIR_R |= UART2_TX_MASK;
    GPIO_PORTB_DR2R_R |= UART1_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R |= UART1_TX_MASK | UART1_RX_MASK;    // enable digital on UART1 pins
    GPIO_PORTB_AFSEL_R |= UART1_TX_MASK | UART1_RX_MASK;  // use peripheral to drive PB0, PB1
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_M | GPIO_PCTL_PB0_M); // clear bits 0-7
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX;
                                                        // select UART1 to drive pins PB0 and PB1: default, added for clarity
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

// Write value to UART1
void putiUart1(uint32_t i)
{
    while (UART1_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART1_DR_R = i;                                  // write character to fifo
}

// Initialize UART2
void initUart2()
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R2;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
    _delay_cycles(3);

    // Unlock PD7
    GPIO_PORTD_LOCK_R = 0x4C4F434B;  // Unlock Port D
    GPIO_PORTD_CR_R |= UART2_TX_MASK | UART2_RX_MASK; // Allow changes to PD7

    // Configure UART2 pins
    GPIO_PORTD_DIR_R |= UART2_TX_MASK;
    GPIO_PORTD_DR2R_R |= UART2_TX_MASK;
    GPIO_PORTD_DEN_R |= UART2_TX_MASK | UART2_RX_MASK;
    GPIO_PORTD_AFSEL_R |= UART2_TX_MASK | UART2_RX_MASK;
    GPIO_PORTD_PCTL_R &= ~(GPIO_PCTL_PD7_M | GPIO_PCTL_PD6_M);
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD7_U2TX | GPIO_PCTL_PD6_U2RX;
}

// Set baud rate as function of instruction cycle frequency
void setUart2BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc << 3) / baudRate;              // calculate divisor (r) in units of 1/128
    divisorTimes128 += 1;                                           // add 1/128 to allow rounding
    divisorTimes128 >>= 1;                                          // change units from 1/128 to 1/64
    UART2_IBRD_R = divisorTimes128 >> 6;                            // set integer value
    UART2_FBRD_R = divisorTimes128 & 0b111111;                      // set fractional value
    UART2_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;                // configure for 8N1 w/ 16-level FIFO
    UART2_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;    // turn-on UART0
}

// Write value to UART2
void putiUart2(uint32_t i)
{
    while (UART2_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART2_DR_R = i;                                  // write character to fifo
}

// Move the robot in a specified direction
void moveWheels(uint8_t frontLeft, uint8_t frontRight, uint8_t backLeft, uint8_t backRight)
{
    putiUart1(frontLeft);
    putiUart2(backLeft);
    waitMicrosecond(100);
    putiUart1(frontRight);
    putiUart2(backRight);
    waitMicrosecond(100);
}

// Move the robot up
void moveUp()
{
    moveWheels((64 + U_FL), (192 - U_FR), (64 + U_BL), (192 - U_BR));
}

// Move the robot sharp left
void moveLeft()
{
    moveWheels((64 - L_FL), (192 - L_FR), (64 - L_BL), (192 - L_BR));
}

// Move the robot down
void moveDown()
{
    moveWheels((64 - D_FL), (192 + D_FR), (64 - D_BL), (192 + D_BR));
}

// Move the robot sharp right
void moveRight()
{
    moveWheels((64 + R_FL), (192 + R_FR), (64 + R_BL), (192 + R_BR));
}

// Stop the robot
void moveStop()
{
    moveWheels(STOP, STOP, STOP, STOP);
}
