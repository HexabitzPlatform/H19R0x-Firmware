/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */

/* Includes ****************************************************************/
#include "BOS.h"

/* Private variables *******************************************************/

/* Private Function Prototypes *********************************************/

/* Main Function ***********************************************************/
int main(void){

	/* Initialize Module &  BitzOS */
	Module_Init();

	/* Don't place your code here */
	for(;;){
	}
}
int speed,Torque ;
uint16_t time ,Time;
int d ;
/***************************************************************************/
/* User Task */
void UserTask(void *argument){
//	MotorSetTorque(1000, 100);

//		HAL_Delay(100);
	/* put your code here, to run repeatedly. */
	while(1){
if (d==1)
{
	Bridge(P2, P4);
d=0; }

if (d==2)
{
	Unbridge(P2, P4);d=0;  }
if (d==3)
{
	Bridge(P1, P3);d=0;  }

if (d==4)
{
	Unbridge(P1, P3); d=0; }






//	 MotorSpeedControl(speed, time);
		//	 MotorSetTorque(Torque, Time);
//		MotorSetTorque(Torque, Time);
//		HAL_Delay(500);
//		MotorSpeedControl(speed, time);
////		MotorMoveToAngle(10, 0.2);
//////		MotorSpeedControl(2000, 750);
//		HAL_Delay(100);
	}
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
