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
	START_MOTOR = 1,
	STOP_MOTOR,
	GET_VELOCITY,
	GET_POSITION,
	SET_VELOCITY,
	SET_POSITION
} Command_Status;
/* Exported functions  ---------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* Commands_Driver_APIS */
/************************ (C) COPYRIGHT Hexabitz *****END OF FILE****/
