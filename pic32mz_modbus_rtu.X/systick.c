#include "systick.h"



void delay(unsigned int delay_ms)
{
    CORETIMER_DelayMs(delay_ms);
}


void delay_us(unsigned int delay_us)
{
    CORETIMER_DelayUs(delay_us);
}

