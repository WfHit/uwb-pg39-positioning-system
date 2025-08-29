#include "delay.h"
#include "stdint.h"

volatile uint32_t time_tick = 0;

void Delay_us(uint32_t us)
{
	time_tick = us;
	while(time_tick>0)
	{}		
}

void Delay_ms(uint32_t ms)
{
	if(ms > UINT32_MAX / 1000)
		ms = UINT32_MAX / 1000;
	
	time_tick = ms * 1000;
	while(time_tick>0)
	{}	
}


