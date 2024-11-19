/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "Commands_Driver_APIs.h"

/* Private variables ---------------------------------------------------------*/
float Position;
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

	// SetPosition(3.26, 1020);
	GetPositionMotor(&Position);

//	SetPositionMotor(3.26, 1020);
	// SetSpeed(1000,500);

	//Test_Function();


	StreamtoPort(0, 2, POS, 10, 10000);
	while (1) {
		HAL_GPIO_TogglePin(_IND_LED_PORT, _IND_LED_PIN);
		// SampletoPort(0, 2, POS);

		HAL_Delay(1000);

	}
}

/*-----------------------------------------------------------*/
