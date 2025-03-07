/*
 * navigation.h
 *
 *  Created on: Apr 26, 2024
 *      Author: madison
 */

#ifndef SONAR_H_
#define SONAR_H_

#include <stdint.h>


/*
 * value from ultasonic at: about a foot is 288
 * 1ft  288
 * 2ft  536
 * 3ft  799
 */

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

uint16_t step;
int32_t echoTime;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initSonar();
void time2_isr();
void echo_isr();

#endif /* NAVIGATION_H_ */
