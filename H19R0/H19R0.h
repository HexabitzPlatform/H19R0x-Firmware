/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H19R0.h
 Description   : Header file for module H19R0.
 Components: UART ports.
 Functions: Motor control interfaces and constants.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H19R0_H
#define H19R0_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H19R0_MemoryMap.h"
#include "H19R0_uart.h"
#include "H19R0_gpio.h"
#include "H19R0_dma.h"
#include "H19R0_inputs.h"
#include "H19R0_eeprom.h"
#include"Commands_Driver_APIs.h"

/* Exported Macros *********************************************************/
#define	MODULE_PN		_H19R0

/* Port-related Definitions */
#define	NUM_OF_PORTS	4
#define P_PROG 			P2		/* ST factory bootloader UART */

/* Define available ports */
#define _P1
#define _P2
#define _P3
#define _P4

/* Define Available USARTs */
#define _USART1
#define _USART2
#define _USART3
#define _USART5
#define _USART6

/* Port-UART mapping */
#define UART_P1 &huart6
#define UART_P2 &huart2
#define UART_P3 &huart3
#define UART_P4 &huart5

/* Port Definitions */
#define USART1_TX_PIN   GPIO_PIN_9
#define USART1_RX_PIN   GPIO_PIN_10
#define USART1_TX_PORT  GPIOA
#define USART1_RX_PORT  GPIOA
#define USART1_AF       GPIO_AF1_USART1

#define USART2_TX_PIN   GPIO_PIN_2
#define USART2_RX_PIN   GPIO_PIN_3
#define USART2_TX_PORT  GPIOA
#define USART2_RX_PORT  GPIOA
#define USART2_AF       GPIO_AF1_USART2

#define USART3_TX_PIN   GPIO_PIN_10
#define USART3_RX_PIN   GPIO_PIN_11
#define USART3_TX_PORT  GPIOB
#define USART3_RX_PORT  GPIOB
#define USART3_AF       GPIO_AF4_USART3

#define USART5_TX_PIN   GPIO_PIN_3
#define USART5_RX_PIN   GPIO_PIN_2
#define USART5_TX_PORT  GPIOD
#define USART5_RX_PORT  GPIOD
#define USART5_AF       GPIO_AF3_USART5

#define USART6_TX_PIN   GPIO_PIN_8
#define USART6_RX_PIN   GPIO_PIN_9
#define USART6_TX_PORT  GPIOB
#define USART6_RX_PORT  GPIOB
#define USART6_AF       GPIO_AF8_USART6

/* Indicator LED */
#define _IND_LED_PORT   GPIOA
#define _IND_LED_PIN    GPIO_PIN_12

/* Module-specific Macro Definitions ***************************************/
#define NUM_MODULE_PARAMS	   1

/* Module_Status Type Definition */
typedef enum {
	H19R0_OK = 0,
	H19R0_ERR_UnknownMessage,
	H19R0_ERR_WrongParams,
	H19R0_ERR_TERMINATED,
	H19R0_ERROR = 255
} Module_Status;

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/

uint8_t MotorTurnOff();
uint8_t MotorMoveToAngle(float Position, float Duration);
uint8_t MotorSpeedControl(uint16_t Time, int16_t Speed);
uint8_t MotorSetTorque(uint16_t Time, int16_t Torque);
uint8_t MotorGetAngle(float *Position);

#endif /* H19R0_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
