/*
 * Commands_Driver_APIS.h
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
#ifndef COMMANDS_DRIVER_API_H_
#define COMMANDS_DRIVER_API_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Module_Status Type Definition */

typedef enum {
	START_MOTOR=0,
	STOP_MOTOR,
	SET_SPEED,
	SET_TORQUE,
	SET_POSITION,
	GET_SPEED,
	GET_POSITION,
	GET_CONTROL_MODE,
	GET_MOVE_DURATON
} Command_Status;


/* Exported macros -----------------------------------------------------------*/
#define DEFAULT_VELOCITY  100u  /* default velocity is 100 rpm */


/* Exported functions  ---------------------------------------------*/

uint8_t SetPosition(float Position, float Duration);
uint8_t GetPosition(float* Position);
uint8_t SetSpeed(uint16_t Time, int16_t Speed);
uint8_t SetTorque(uint16_t Time, int16_t Torque);
uint8_t GetMoveDuration(float* Duration);
uint8_t Test_Function();

#ifdef __cplusplus
}
#endif

#endif /* Commands_Driver_APIS */
/************************ (C) COPYRIGHT Hexabitz *****END OF FILE****/
