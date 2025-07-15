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
volatile uint8_t motorTestSelector = 20;

/*-----------------------------------------------------------*/
float pose[10];
uint8_t h[6] = {1,2,3,4,5,6};
float position =0.0f;  // ≈ 90 degrees in radians
float duration = 0.0f;   // move duration in seconds
    uint8_t result = 0;
/* User Task */
void UserTask(void *argument) {

	// put your code here, to run repeatedly.

	while (1) {
//		SetPositionMotor(1.56, 1);
////		SetSpeedMotor(50, 750);
//		HAL_Delay(1000);
	}
	}


/*-----------------------------------------------------------*/
