#ifndef DELAY_H
#define DELAY_H

#include "stm32f10x.h"

void timing_delay_decrement(void);
void delay_ms(__IO uint32_t nTime);

#endif // DELAY_H
