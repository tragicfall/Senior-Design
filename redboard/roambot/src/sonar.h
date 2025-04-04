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

uint16_t step0;
int32_t echoTime0;
uint16_t step1;
int32_t echoTime1;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initSonar();
void trig0_isr();
void echo0_isr();
void trig1_isr();
void echo1_isr();

#endif /* NAVIGATION_H_ */
