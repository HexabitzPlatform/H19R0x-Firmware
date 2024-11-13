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

uint8_t command = 1;
float arg11 = 5.3, arg12 = 3.6;

/* Local Functions Definitions */
uint8_t ConvertFloatTwoBytes(float *arg, uint8_t *fourBytes);
uint8_t PrepareMessage(uint8_t command);
uint8_t SendCommand(uint8_t commands, float arg1, float arg2);
uint8_t ReceiveMessage();
uint8_t ProccessReceivedMessage(uint8_t *commands, float *arg1, float *arg2);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	isReceivedMEG = 1;
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
		fourBytes[0] = (*(ptr + 0));
		fourBytes[1] = (*(ptr + 1));
		fourBytes[2] = (*(ptr + 2));
		fourBytes[3] = (*(ptr + 3));
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
	SendingMessage[0] = "H";
	SendingMessage[1] = "Z";

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
 * @param3: *arg2 :  pointer of second argument in received result .
 */
uint8_t ProccessReceivedMessage(uint8_t *commands, float *arg1, float *arg2) {
	*commands = ReceivedMessage[2];
	uint32_t temp = 0;
	temp = ((ReceivedMessage[3] << 0) | (ReceivedMessage[4] << 8)
			| (ReceivedMessage[5] << 16) | ReceivedMessage[6] << 24);
	*arg1 = *((float*) &temp);

	temp = ((ReceivedMessage[7] << 0) | (ReceivedMessage[8] << 8)
			| (ReceivedMessage[9] << 16) | ReceivedMessage[10] << 24);
	*arg2 = *((float*) &temp);

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
	SendCommand(GET_POSITION, 0.0, 0.0);
	while(1){

		if(isReceivedMEG==1)
		{
			isReceivedMEG = 0;
			break;
		}
	}
	ProccessReceivedMessage(&command, Position, &arg12);

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
	ProccessReceivedMessage(&command, Duration, &arg12);

	return 0;
}


/************************ (C) COPYRIGHT Hexabitz *****END OF FILE****/
