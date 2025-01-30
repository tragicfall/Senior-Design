// UART Library
// Andrew Howard

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:

// Hardware configuration:
// UART Interface:
//   U1TX (PB1) and U1RX (PB0) are connected to the 1st controller
//   U2TX (PD7) and U2RX (PD6) are connected to the 2nd controller

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef UART_H_
#define UART_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initUart1();
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc);
void putiUart1(uint32_t i);

void initUart2();
void setUart2BaudRate(uint32_t baudRate, uint32_t fcyc);
void putiUart2(uint32_t i);

#endif
