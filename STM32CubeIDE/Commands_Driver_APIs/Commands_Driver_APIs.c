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
uint8_t FirstBytesSentMSG[4], SecondBytesSentMSG[4];
uint8_t ReceivedMessage[11];
uint8_t isReceivedMEG = 0;
uint8_t FirstBytesRevdMSG[4];
uint8_t SecondBytesRevdMSG[4];

/* Local Functions Definitions */
uint8_t ConvertFloatTwoBytes(float arg, uint8_t *fourBytes);
uint8_t ConvertUint16TwoBytes(uint16_t arg, uint8_t *twoBytes);
uint8_t ConvertInt16TwoBytes(int16_t arg, uint8_t *twoBytes);
uint8_t PrepareMessage(uint8_t command);
uint8_t SendCommand(uint8_t commands, float arg1, float arg2);
uint8_t ReceiveMessage();
uint8_t ProcessReceivedCommand(uint8_t *commands);
uint8_t ProcessReceivedFloat(uint8_t *ReceivedBytes, float *arg1);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	isReceivedMEG = 1;
//	HAL_UART_Transmit_IT(&huart1, ReceivedMessage, 6);
	FirstBytesRevdMSG[0] = ReceivedMessage[3];
	FirstBytesRevdMSG[1] = ReceivedMessage[4];
	FirstBytesRevdMSG[2] = ReceivedMessage[5];
	FirstBytesRevdMSG[3] = ReceivedMessage[6];

	SecondBytesRevdMSG[0] = ReceivedMessage[7];
	SecondBytesRevdMSG[1] = ReceivedMessage[8];
	SecondBytesRevdMSG[2] = ReceivedMessage[9];
	SecondBytesRevdMSG[3] = ReceivedMessage[10];
}

/**************************************************************************/
/* Local Functions  *******************************************************/
/**************************************************************************/

/**
 * @brief get value of four bytes n which float variable is stored
 * @param1: *arg :  pointer of float value.
 * @param2: *fourBytes :  pointer of four uint8_t bytes.
 */
uint8_t ConvertFloatTwoBytes(float arg, uint8_t *fourBytes) {
	uint8_t *ptr, i;

	ptr = (uint8_t*) &(arg);
	/* casting float variable into four bytes */
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
uint8_t ConvertUint16TwoBytes(uint16_t arg, uint8_t *twoBytes) {
	uint8_t *ptr1, i;
	ptr1 = (uint8_t*) &(arg);
	/* casting uint16_t variable into two bytes */
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
uint8_t ConvertInt16TwoBytes(int16_t arg, uint8_t *twoBytes) {
	uint8_t *ptr1, i;
	ptr1 = (uint8_t*) &(arg);
	/* casting int16_t variable into two bytes */
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

	SendingMessage[3] = FirstBytesSentMSG[0];  // LS
	SendingMessage[4] = FirstBytesSentMSG[1];
	SendingMessage[5] = FirstBytesSentMSG[2];
	SendingMessage[6] = FirstBytesSentMSG[3];

	SendingMessage[7] = SecondBytesSentMSG[0];
	SendingMessage[8] = SecondBytesSentMSG[1];
	SendingMessage[9] = SecondBytesSentMSG[2];
	SendingMessage[10] = SecondBytesSentMSG[3];

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
 * @param1: *ReceivedBytes : pointer of Received Bytes in the received message .
 * @param2: *arg1 :  pointer of argument in received result .
 */
uint8_t ProcessReceivedFloat(uint8_t *ReceivedBytes, float *arg1) {

	uint32_t temp = 0;
	/* shifting four bytes and casting them into float */
	temp = ((ReceivedBytes[0] << 0) | (ReceivedBytes[1] << 8)
			| (ReceivedBytes[2] << 16) | ReceivedBytes[3] << 24);
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

	ConvertFloatTwoBytes(Position, FirstBytesSentMSG);
	ConvertFloatTwoBytes(Duration, SecondBytesSentMSG);
	PrepareMessage(SET_POSITION);
	HAL_UART_Transmit_IT(&huart1, SendingMessage, 11);

	return 0;
}

/**********************************************************************/

/**
 * @brief returns the current position of Motor 1.
 *  	get position from initial position in radian
 *   */
uint8_t GetPosition(float *Position) {

	ReceiveMessage();
	PrepareMessage(GET_POSITION);
	HAL_UART_Transmit_IT(&huart1, SendingMessage, 11);
	while (1) {

		if (isReceivedMEG == 1) {
			isReceivedMEG = 0;
			break;
		}
	}
	uint8_t command;
	ProcessReceivedCommand(&command);
	if (command == GET_POSITION) {

		ProcessReceivedFloat(SecondBytesRevdMSG, Position);
		 // only for verifying results

		  	ConvertFloatTwoBytes(*Position, FirstBytesSentMSG);
		  	ConvertFloatTwoBytes(*Position, SecondBytesSentMSG);

		  	PrepareMessage(command);
		  	HAL_UART_Transmit_IT(&huart1, SendingMessage, 11);


	}

	return 0;
}

/**********************************************************************/
/**
 * @brief Programs a speed command for Motor in the given @p Duration Time.
 *
 * @param  Speed Target mechanical angle reference at the end of the movement.
 *         This value represents the final position expressed in rpm.
 * @param  Duration of the movement expressed in ms.
 */

uint8_t SetSpeed(uint16_t Time, int16_t Speed) {
	ConvertUint16TwoBytes(Time, FirstBytesSentMSG);
	ConvertInt16TwoBytes(Speed, SecondBytesSentMSG);
	PrepareMessage(SET_SPEED);
	HAL_UART_Transmit_IT(&huart1, SendingMessage, 11);
}

/**********************************************************************/
/**
 * @brief Programs a torque command for Motor in the given @p Duration Time.
 *
 * @param  Mechanical motor torque reference at the end of the ramp.
  *         This value represents actually the Iq current expressed in digit.
 * @param  Duration of the movement expressed in ms.
 */
uint8_t SetTorque(uint16_t Time, int16_t Torque) {
	ConvertUint16TwoBytes(Time, FirstBytesSentMSG);
	ConvertInt16TwoBytes(Torque, SecondBytesSentMSG);
	PrepareMessage(SET_TORQUE);
	HAL_UART_Transmit_IT(&huart1, SendingMessage, 11);
}

/**********************************************************************/
/**
 * @brief the total movement duration to reach the target position of Motor.
 * * @param	Duration: pointer of duration
 *   */
uint8_t GetMoveDuration(float *Duration) {
	ReceiveMessage();
	PrepareMessage(GET_MOVE_DURATON);
	HAL_UART_Transmit_IT(&huart1, SendingMessage, 11);
	while (1) {

		if (isReceivedMEG == 1) {
			isReceivedMEG = 0;
			break;
		}
	}
	uint8_t command;
	ProcessReceivedCommand(&command);
	if (command == GET_MOVE_DURATON) {
		ProcessReceivedFloat(FirstBytesRevdMSG, Duration);
	}

	return 0;
}

/**********************************************************************/
/**
 * @brief the total movement duration to reach the target position of Motor.
 * * @param	Duration: pointer of duration
 *   */
uint8_t GetControlMode(uint8_t* Mode)
{
	ReceiveMessage();
	PrepareMessage(GET_CONTROL_MODE);
	HAL_UART_Transmit_IT(&huart1, SendingMessage, 11);
	while (1) {

		if (isReceivedMEG == 1) {
			isReceivedMEG = 0;
			break;
		}
	}
	uint8_t command;
	ProcessReceivedCommand(&command);
	if (command == GET_CONTROL_MODE) {
		*Mode = FirstBytesRevdMSG[0];
	}

	return 0;
}

uint8_t Test_Function() {
	return 0;
}

/************************ (C) COPYRIGHT Hexabitz *****END OF FILE****/
