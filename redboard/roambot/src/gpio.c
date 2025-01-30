// GPIO Library
// Andrew Howard

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "gpio.h"

// PortD masks
#define PD0_RIGHT (0b0001)
#define PD1_DOWN  (0b0010)
#define PD2_LEFT  (0b0100)
#define PD3_UP    (0b1000)
#define PD_ALL    (0b1111)

void initGPIO()
{
    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
    _delay_cycles(3);

    GPIO_PORTD_LOCK_R = 0x4C4F434B;  // Unlock Port D
    GPIO_PORTD_CR_R |= PD_ALL;       // Allow changes to PD0 and PD1

    GPIO_PORTD_DIR_R |= PD_ALL;
    GPIO_PORTD_DEN_R |= PD_ALL;
    GPIO_PORTD_PDR_R |= PD_ALL;
}

// return the appropriate control value by polling the GPIO port 9 times
uint8_t getControls()
{
    uint8_t up_count = 0;
    uint8_t left_count = 0;
    uint8_t down_count = 0;
    uint8_t right_count = 0;
    uint8_t controls = 0;
    uint8_t i;

    for (i=0; i<9; i++)
    {
        up_count    += (GPIO_PORTD_DATA_R & PD3_UP) >> 3;
        left_count  += (GPIO_PORTD_DATA_R & PD2_LEFT) >> 2;
        down_count  += (GPIO_PORTD_DATA_R & PD1_DOWN) >> 1;
        right_count += (GPIO_PORTD_DATA_R & PD0_RIGHT) >> 0;
    }

    controls |= (up_count > 4) << 3;
    controls |= (left_count > 4) << 2;
    controls |= (down_count > 4) << 1;
    controls |= (right_count > 4) << 0;

    return controls;
}
