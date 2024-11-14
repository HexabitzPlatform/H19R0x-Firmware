/*
 * Commands_Driver_APIS.C
 * Description: Sending commands and receiving message from BLDC motor driver.
 *  Created on: Oct 30, 2024
 *      Author: Muhammad Alhaddad @ Hexabitz
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 Hexabitz.
 * All rights reserved.
 *
 ******************************************************************************
 */

#include"Commands_Driver_APIs.h"
#include"H19R0_uart.h"
uint8_t SendingMessage[11];
uint8_t var1[4], var2[4];
uint8_t ReceivedMessage[11];
uint8_t isReceivedMEG = 0;
uint8_t FirstReceivedBytes[4];
uint8_t SecondReceivedBytes[4];

uint8_t command = 1;
float arg11 = 5.3, arg12 = 3.6;

/* Local Functions Definitions */
uint8_t ConvertFloatTwoBytes(float *arg, uint8_t *fourBytes);
uint8_t ConvertUint16TwoBytes(uint16_t *arg, uint8_t *twoBytes);
uint8_t PrepareMessage(uint8_t command);
uint8_t SendCommand(uint8_t commands, float arg1, float arg2);
uint8_t ReceiveMessage();
uint8_t ProcessReceivedCommand(uint8_t *commands);
uint8_t ProcessReceivedFloat(uint8_t* FirstReceivedBytes, float *arg1);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	isReceivedMEG = 1;
//	HAL_UART_Transmit_IT(&huart1, ReceivedMessage, 6);
	FirstReceivedBytes[0]=ReceivedMessage[3];
	FirstReceivedBytes[1]=ReceivedMessage[4];
	FirstReceivedBytes[2]=ReceivedMessage[5];
	FirstReceivedBytes[3]=ReceivedMessage[6];

	SecondReceivedBytes[0]=ReceivedMessage[7];
	SecondReceivedBytes[1]=ReceivedMessage[8];
	SecondReceivedBytes[2]=ReceivedMessage[9];
	SecondReceivedBytes[3]=ReceivedMessage[10];
}

/**************************************************************************/
/* Local Functions  *******************************************************/
/**************************************************************************/

/**
 * @brief get value of four bytes n which float variable is stored
 * @param1: *arg :  pointer of float value.
 * @param2: *fourBytes :  pointer of four uint8_t bytes.
 */
uint8_t ConvertFloatTwoBytes(float *arg, uint8_t *fourBytes) {
	uint8_t *ptr, i;

	ptr = (uint8_t*) &(*arg);

	for (i = 0; i < 4; i++) {
		fourBytes[i] = (*(ptr + i));
	}
	return 0;
}

/**********************************************************************/

/**
 * @brief get value of two bytes in which uint16_t variable is stored
 * @param1: *arg :  pointer of uint16_t value.
 * @param2: *twoBytes :  pointer of four uint8_t bytes.
 */
uint8_t ConvertUint16TwoBytes(uint16_t *arg, uint8_t *twoBytes) {
	uint8_t *ptr1;
	ptr1 = (uint8_t*) &(*arg);

	for (i = 0; i < 2; i++) {
		twoBytes[i] = (*(ptr1 + i));
	}

	return 0;
}

/**********************************************************************/

/**
 * @brief get value of two bytes in which int16_t variable is stored
 * @param1: *arg :  pointer of int16_t value.
 * @param2: *twoBytes :  pointer of four uint8_t bytes.
 */
uint8_t ConvertInt16TwoBytes(int16_t *arg, uint8_t *twoBytes) {
	uint8_t *ptr1;
	ptr1 = (uint8_t*) &(*arg);

	for (i = 0; i < 2; i++) {
		twoBytes[i] = (*(ptr1 + i));
	}

	return 0;
}

/**********************************************************************/

/**
 * @brief preparing message command sent to stspin
 * so it will consists of 11 bytes
 * "H" + "Z" + command (one byte typedef) +
 * + four bytes first argument + four bytes second argument
 * @param1: command :  command need to be sent .
 */
uint8_t PrepareMessage(uint8_t command) {

	SendingMessage[0] = 'H';
	SendingMessage[1] = 'Z';

	SendingMessage[2] = command;

	SendingMessage[3] = var1[0];
	SendingMessage[4] = var1[1];
	SendingMessage[5] = var1[2];
	SendingMessage[6] = var1[3];

	SendingMessage[7] = var2[0];
	SendingMessage[8] = var2[1];
	SendingMessage[9] = var2[2];
	SendingMessage[10] = var2[3];

	return 0;
}

/**********************************************************************/

/**
 * @brief sending command using UART1 port to stspin
 * @param1: command :  command need to be sent .
 * @param2: arg1 :  first argument in command .
 * @param3: arg2 :  second argument in command .
 */
uint8_t SendCommand(uint8_t commands, float arg1, float arg2) {
	ConvertFloatTwoBytes(&arg1, var1);
	ConvertFloatTwoBytes(&arg2, var2);
	PrepareMessage(commands);
	HAL_UART_Transmit_IT(&huart1, SendingMessage, 11);

	return 0;
}

/**********************************************************************/

/**
 * @brief start receiving message on UART1
 */
uint8_t ReceiveMessage() {
	HAL_UART_Receive_IT(&huart1, ReceivedMessage, 11);
}

/**********************************************************************/

/**
 * @brief processing received message from stspin
 * @param1: *command : pointer of command to which the received message is related .
 * @param2: *arg1 :  pointer of first argument in received result .
 */
uint8_t ProcessReceivedCommand(uint8_t *commands) {
	*commands = ReceivedMessage[2];

	return 0;

}

/**********************************************************************/

/**
 * @brief processing received message from stspin
 * @param1: *FirstReceivedBytes : pointer of Received Bytes in the received message .
 * @param2: *arg1 :  pointer of argument in received result .
 */
uint8_t ProcessReceivedFloat(uint8_t* FirstReceivedBytes, float *arg1) {

	uint32_t temp = 0;
	temp = ((FirstReceivedBytes[0] << 24) | (FirstReceivedBytes[1] << 16)
			| (FirstReceivedBytes[2] << 8) | FirstReceivedBytes[3] << 0);
	*arg1 = *((float*) &temp);

	return 0;

}


/**************************************************************************/
/* Exported functions  ****************************************************/
/**************************************************************************/

/**
  * @brief Programs a position command for Motor in the given @p Duration time.
  *
  * @param  Position Target mechanical angle reference at the end of the movement.
  *         This value represents the final position expressed in radian.
  * @param  Duration of the movement expressed in seconds.
  */
uint8_t SetPosition(float Position, float Duration) {

	SendCommand(SET_POSITION, Position, Duration);

	return 0;
}

/**********************************************************************/

/**
 * @brief returns the current position of Motor 1.
 *  	get position from initial position in radian
 *   */
uint8_t GetPosition(float* Position){

	ReceiveMessage();
	SendCommand(GET_POSITION, 6.0, 5.3);
	while(1){

		if(isReceivedMEG==1)
		{
			isReceivedMEG = 0;
			break;
		}
	}
	ProcessReceivedCommand(&command);
	if(command==GET_POSITION)
	{

		ProcessReceivedFloat(FirstReceivedBytes, Position);
		// only for test
	//	HAL_UART_Transmit_IT(&huart1, FirstReceivedBytes, 4);
		ConvertFloatTwoBytes(Position, var1);
		ConvertFloatTwoBytes(Position, var2);
		PrepareMessage(command);
		HAL_UART_Transmit_IT(&huart1, SendingMessage, 11);
	}

	return 0;
}

/**********************************************************************/
/**
 * @brief the total movement duration to reach the target position of Motor.
 * * @param	Duration: pointer of duration
 *   */
uint8_t GetMoveDuration(float *Duration) {
	ReceiveMessage();
	SendCommand(GET_MOVE_DURATON, 0.0, 0.0);
	while (1) {

		if (isReceivedMEG == 1) {
			isReceivedMEG = 0;
			break;
		}
	}
	ProcessReceivedCommand(&command);
	if(command==GET_MOVE_DURATON)
	{
		ProcessReceivedFloat(FirstReceivedBytes, Duration);
	}

	return 0;
}

uint8_t Test_Function() {
	return 0;
}


/************************ (C) COPYRIGHT Hexabitz *****END OF FILE****/
