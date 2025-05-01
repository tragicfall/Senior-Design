// GPIO Library (Header)
// File: gpio.h
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

// Hardware configuration
// GPIO PIN PD0 - Input to go Right
// GPIO PIN PD1 - Input to go Down
// GPIO PIN PD2 - Input to go Left
// GPIO PIN PD3 - Input to go Up

//-----------------------------------------------------------------------------
// Preprocessor Directive
//-----------------------------------------------------------------------------

#ifndef GPIO_H_
#define GPIO_H_

//-----------------------------------------------------------------------------
// Device Includes
//-----------------------------------------------------------------------------

#include <stdint.h>

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void    initGPIO();        // Initialize GPIO Port D for detecting controls
uint8_t getControlsGPIO(); // Get controls from GPIO Port D

#endif
