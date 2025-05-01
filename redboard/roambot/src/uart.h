// UART Library (Header)
// File: uart.h
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
// Preprocessor Directive
//-----------------------------------------------------------------------------

#ifndef UART_H_
#define UART_H_

//-----------------------------------------------------------------------------
// Device Includes
//-----------------------------------------------------------------------------

#include <stdint.h>

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initUart1();                                         // Initialize UART1 for 115200 baud rate, 8 data bits, no parity, 1 stop bit
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc);  // Set the baud rate for UART1
void putiUart1(uint32_t i);                               // Send an integer to UART1

void initUart2();                                         // Initialize UART2 for 115200 baud rate, 8 data bits, no parity, 1 stop bit
void setUart2BaudRate(uint32_t baudRate, uint32_t fcyc);  // Set the baud rate for UART2
void putiUart2(uint32_t i);                               // Send an integer to UART2

// Move wheels with specified speed (0-255) for each wheel
void moveWheels(uint8_t frontLeft, uint8_t frontRight, uint8_t backLeft, uint8_t backRight); 

void moveUp();    // Move forward
void moveLeft();  // Move left
void moveDown();  // Move backward
void moveRight(); // Move right
void moveStop();  // Stop moving

#endif
