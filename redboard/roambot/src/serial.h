// Serial Library (Header)
// File: serial.h
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
// Preprocessor Directive
//-----------------------------------------------------------------------------

#ifndef SERIAL_H_
#define SERIAL_H_

//-----------------------------------------------------------------------------
// Device Includes
//-----------------------------------------------------------------------------

#include <stdint.h>

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initSerial();                                       // Initialize UART3
void setUart3BaudRate(uint32_t baudRate, uint32_t fcyc); // Set baud rate as function of instruction cycle frequency
uint32_t getControlsSerial(uint32_t current_controls);   // Get controls from UART3

#endif
