// GPIO Library (Definition)
// File: gpio.c
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
// Device Includes
//-----------------------------------------------------------------------------

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "gpio.h"

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

#define PD0_UP    (0b0001)
#define PD1_LEFT  (0b0010)
#define PD2_DOWN  (0b0100)
#define PD3_RIGHT (0b1000)
#define PD_ALL    (0b1111)

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize GPIO Port D for detecting controls
void initGPIO()
{
    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
    _delay_cycles(3);

    GPIO_PORTD_LOCK_R = 0x4C4F434B;  // Unlock Port D
    GPIO_PORTD_CR_R |= PD_ALL;       // Allow changes to PD0 and PD1

    GPIO_PORTD_DIR_R &= ~PD_ALL;
    GPIO_PORTD_DEN_R |=  PD_ALL;
    GPIO_PORTD_PDR_R |=  PD_ALL;
}

// Read the controls from the GPIO Port D (9 samples)
uint8_t getControlsGPIO()
{
    uint8_t up_count = 0;
    uint8_t left_count = 0;
    uint8_t down_count = 0;
    uint8_t right_count = 0;
    uint8_t controls = 0;
    uint8_t i;

    for (i=0; i<9; i++)
    {
        up_count    += (GPIO_PORTD_DATA_R & PD0_UP) >> 0;
        left_count  += (GPIO_PORTD_DATA_R & PD1_LEFT) >> 1;
        down_count  += (GPIO_PORTD_DATA_R & PD2_DOWN) >> 2;
        right_count += (GPIO_PORTD_DATA_R & PD3_RIGHT) >> 3;
    }

    controls |= (up_count > 4) << 0;
    controls |= (left_count > 4) << 1;
    controls |= (down_count > 4) << 2;
    controls |= (right_count > 4) << 3;

    return controls;
}
