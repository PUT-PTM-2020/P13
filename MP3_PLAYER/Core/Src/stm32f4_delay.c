#include "stm32f4_delay.h"

static __IO uint32_t TimingDelay = 0;
static __IO uint32_t Time = 0;
uint8_t DELAY_Initialized = 0;

void Delay(__IO uint32_t nTime) 
{
	TimingDelay = nTime;

	while (TimingDelay != 0);
}

void Delayms(__IO uint32_t nTime) 
{
	TimingDelay = 1000 * nTime;

	while (TimingDelay != 0);
}

#ifdef KEIL_IDE
void TimingDelay_Decrement(void) 
{
	Time++;
	if (Time2 != 0x00) 
	{
		Time2--;
	}
	if (TimingDelay != 0x00) 
	{
		TimingDelay--;
	}
}
#else
void SysTick_Handler(void) 
{
	Time++;
	if (Time2 != 0x00) 
	{
		Time2--;
	}
	if (TimingDelay != 0x00) 
	{
		TimingDelay--;
	}
}
#endif

void DELAY_Init(void) 
{
	if (DELAY_Initialized) 
	{
		return;
	}
	RCC_HSEConfig(RCC_HSE_ON);
	while (!RCC_WaitForHSEStartUp());
	/* Set Systick interrupt every 1us */
	if (SysTick_Config(SystemCoreClock / 1000000)) 
	{
		/* Capture error */
		while (1);
	}
	DELAY_Initialized = 1;
}

uint32_t DELAY_Time(void) 
{
	return Time;
}

uint32_t DELAY_Time2(void) 
{
	return Time2;
}

void DELAY_SetTime(uint32_t time) 
{
	Time = time;
}

void DELAY_SetTime2(uint32_t time) 
{
	Time2 = time;
}

void DELAY_EnableSystick(void) 
{
	if (!DELAY_Initialized) 
	{
		return;
	}
	
	/* Wlaczenie przerwan na systicku */  
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}

void DELAY_DisableSystick(void) 
{
	/* Wylaczenie Systicka */
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}
