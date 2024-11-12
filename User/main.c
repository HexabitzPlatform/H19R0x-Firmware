/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/
uint8_t MSG_P[5]="Hello";
/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int main(void) {

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for (;;) {
	}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument) {

	// put your code here, to run repeatedly.
	uint8_t d = 0;
	while (1) {

		HAL_GPIO_WritePin(_IND_LED_PORT, _IND_LED_PIN, SET);
		HAL_GPIO_WritePin(_IND_LED_PORT, _IND_LED_PIN, RESET);
		HAL_GPIO_WritePin(_IND_LED_PORT, _IND_LED_PIN, SET);
		HAL_GPIO_WritePin(_IND_LED_PORT, _IND_LED_PIN, RESET);
		HAL_GPIO_WritePin(_IND_LED_PORT, _IND_LED_PIN, SET);



	}
}

/*-----------------------------------------------------------*/
