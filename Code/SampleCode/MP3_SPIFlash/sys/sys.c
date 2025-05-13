#include "sys.h"
//#include "GPCM1Fx.h"
#include "GPCM3_FM1.h"
extern uint32_t SystemCoreClock;
volatile uint32_t tick = 0;

void SysTick_Handler(void)
{
    tick++;
}

void delay(uint32_t time, uint32_t load)
{
    tick = 0;

    SysTick_Config(load);

    do
		{
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    } while (tick < time);

    SysTick->CTRL = 0;
}

void delay_ms(uint32_t time)
{
    delay(time, SystemCoreClock / 1000);
}

void delay_us(uint32_t time)
{
    delay(time, SystemCoreClock / 1000000);
}
