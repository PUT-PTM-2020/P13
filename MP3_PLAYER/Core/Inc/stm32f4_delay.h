#ifndef DELAY_
#define DELAY_ 120

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
//#include "defines.h"

static __IO uint32_t TimingDelay;

/*
		Te zmienne moga byc uzywane w main
		Sa automatycznie zwiekszane kiedy systick zglosi przerwanie
 */
static __IO uint32_t Time;
static __IO uint32_t Time2;

/*
		Wprowadzamy opóznienie
		Podawane jest w mikrosekundach
		Funkcja nic nie zwraca
 */
void Delay(__IO uint32_t nTime);

/**
 *
 * @brief  Inserts a delay time.
 * @param  nTime: specifies the delay time length, in milliseconds.
 * @retval None
 */
void Delayms(__IO uint32_t nTime);

/*
		Inicjalizacja ustawien timera dla Delay
 */
void DELAY_Init(void);

/*
 * Get the Time variable value
 */
uint32_t DELAY_Time(void);

/*
 * Set value for Time variable
 */
void DELAY_SetTime(uint32_t time);

/*
 * Get the Time2 variable value
 */
uint32_t DELAY_Time2(void);

/*
 * Set value for Time variable
 */
void DELAY_SetTime2(uint32_t time);

/**
 * Re-enable Systick. It has to be configured before with DELAY_Init();
 */
void DELAY_EnableSystick(void);

/**
 * Disable Systick. Usef when going into sleep mode, so systick will not
 */
void DELAY_DisableSystick(void);

#endif
