/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H19R0_timers.c
 Description: Configures timers and watchdog for delays.
 Timers: TIM16 (usec), TIM17 (msec)
 IWDG: 500 ms timeout watchdog.
 */

/* Includes ****************************************************************/
#include "BOS.h"

/* Exported Functions ******************************************************/
void TIM_USEC_Init(void);
void TIM_MSEC_Init(void);
void MX_IWDG_Init(void);

/* Exported Variables ******************************************************/
TIM_HandleTypeDef htim16; /* micro-second delay counter */
TIM_HandleTypeDef htim17; /* milli-second delay counter */
IWDG_HandleTypeDef hiwdg;

/***************************************************************************/
/* IWDG init function */
void MX_IWDG_Init(void) {
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
	hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
	hiwdg.Init.Reload = 1999;
	HAL_IWDG_Init(&hiwdg);
}

/***************************************************************************/
/* Micro-seconds timebase init function - TIM16 (16-bit) */
void TIM_USEC_Init(void) {
	__TIM16_CLK_ENABLE();

	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 47;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 0XFFFF;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim16);
	HAL_TIM_Base_Start(&htim16);
}

/***************************************************************************/
/* Milli-seconds timebase init function - TIM17 (16-bit) */
void TIM_MSEC_Init(void) {
	__TIM17_CLK_ENABLE();

	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 47999;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 0xFFFF;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim17);
	HAL_TIM_Base_Start(&htim17);
}

/***************************************************************************/
/* Load and start micro-second delay counter */
void StartMicroDelay(uint16_t Delay) {
	uint32_t t0 = 0;

	portENTER_CRITICAL();
	if (Delay) {
		t0 = htim16.Instance->CNT;
		while (htim16.Instance->CNT - t0 <= Delay) {}
	}
	portEXIT_CRITICAL();
}

/***************************************************************************/
/* Load and start milli-second delay counter */
void StartMilliDelay(uint16_t Delay) {
	uint32_t t0 = 0;

	portENTER_CRITICAL();
	if (Delay) {
		t0 = htim17.Instance->CNT;
		while (htim17.Instance->CNT - t0 <= Delay) {}
	}
	portEXIT_CRITICAL();
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
