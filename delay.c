#include "delay.h"
#include "stm32f10x.h"


static __IO uint32_t timing_delay; /* variable updated by systick interrupt handler */

void timing_delay_decrement(void)
{
    if (timing_delay != 0x00) {
        timing_delay--;
    }
}

void delay_ms(__IO uint32_t n_time)
{
    timing_delay = n_time;
    while(timing_delay != 0);
}
