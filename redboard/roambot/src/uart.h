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

void initUart1();
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc);
void putiUart1(uint32_t i);

void initUart2();
void setUart2BaudRate(uint32_t baudRate, uint32_t fcyc);
void putiUart2(uint32_t i);

void move(uint8_t frontLeft, uint8_t frontRight, uint8_t backLeft, uint8_t backRight);
void moveUp();
void moveLeft();
void moveDown();
void moveRight();
void moveUpLeft();
void moveUpRight();
void moveDownLeft();
void moveDownRight();
void moveStop();

#endif
