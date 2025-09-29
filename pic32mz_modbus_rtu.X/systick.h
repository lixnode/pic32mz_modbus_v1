/* 
 * File:   systick.h
 * Author: thanho
 *
 * Created on September 28, 2025, 8:04 PM
 */

#ifndef SYSTICK_H
#define	SYSTICK_H

#include <stdlib.h>
#include <stdint.h>
#include "../src/config/default/peripheral/coretimer/plib_coretimer.h"


void delay(unsigned int delay_ms);
void delay_us(unsigned int delay_us);


#endif	/* SYSTICK_H */

