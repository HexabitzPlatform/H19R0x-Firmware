/**
 * @file main.c
 * @brief General template for Hexabitz module H19R0, managing system initialization and motor control.
 * @details Initializes UART1-6, DMA channels, and timers for motor control. Provides CLI commands
 *          for motor control: stop_motor, set_position, set_speed, set_torque. Processes messages
 *          for motor position, speed, and torque control. Manages power modes and flash storage.
 * @author Hexabitz
 * @copyright (C) 2017-2025 Hexabitz
 */

/* Includes ****************************************************************/
#include "BOS.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

/* Exported Typedef ********************************************************/
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Module Parameters */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] = { 0 };

/* Private Function Prototypes *********************************************/
void Module_Peripheral_Init(void);

/* Create CLI Commands *****************************************************/
static portBASE_TYPE CLI_MotorTurnOffCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE CLI_MotorMoveToAngleCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE CLI_MotorSpeedControlCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE CLI_MotorSetTorqueCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/* CLI Command Structure ***************************************************/

/* CLI command structure : MotorTurnOff */
const CLI_Command_Definition_t CLI_MotorTurnOffCommandDefinition = {
    (const int8_t *)"stop_motor",
    (const int8_t *)"stop_motor:\r\nStops the motor.\n\r",
    CLI_MotorTurnOffCommand, /* The function to run. */
    0 /* No parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : MotorMoveToAngle */
const CLI_Command_Definition_t CLI_MotorMoveToAngleCommandDefinition = {
    (const int8_t *)"set_angle",
    (const int8_t *)"set_angle:\r\nSets motor angle and duration.\r\nParameters:\r\n1) Angle: float\r\n2) Duration: float\n\r",
    CLI_MotorMoveToAngleCommand, /* The function to run. */
    2 /* Two parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : MotorSpeedControl */
const CLI_Command_Definition_t CLI_MotorSpeedControlCommandDefinition = {
    (const int8_t *)"set_speed",
    (const int8_t *)"set_speed:\r\nSets motor speed and duration.\r\nParameters:\r\n1) Time: ms\r\n2) Speed: int\n\r",
    CLI_MotorSpeedControlCommand, /* The function to run. */
    2 /* Two parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : MotorSetTorque */
const CLI_Command_Definition_t CLI_MotorSetTorqueCommandDefinition = {
    (const int8_t *)"set_torque",
    (const int8_t *)"set_torque:\r\nSets motor torque and duration.\r\nParameters:\r\n1) Time: ms\r\n2) Torque: int\n\r",
    CLI_MotorSetTorqueCommand, /* The function to run. */
    2 /* Two parameters are expected. */
};
/***************************************************************************/
/************************ Private function Definitions *********************/
/***************************************************************************/
/* @brief  System Clock Configuration
 *         This function configures the system clock as follows:
 *            - System Clock source            = PLL (HSE)
 *            - SYSCLK(Hz)                     = 64000000
 *            - HCLK(Hz)                       = 64000000
 *            - AHB Prescaler                  = 1
 *            - APB1 Prescaler                 = 1
 *            - HSE Frequency(Hz)              = 8000000
 *            - PLLM                           = 1
 *            - PLLN                           = 16
 *            - PLLP                           = 2
 *            - Flash Latency(WS)              = 2
 *            - Clock Source for UART1,UART2,UART3 = 16MHz (HSI)
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 16;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}
/***************************************************************************/
/* Enable stop mode regarding only UART1, UART2, and UART3 */
BOS_Status EnableStopModebyUARTx(uint8_t port) {
    UART_WakeUpTypeDef WakeUpSelection;
    UART_HandleTypeDef *huart = GetUart(port);

    if ((huart->Instance == USART1) || (huart->Instance == USART2) || (huart->Instance == USART3)) {
        /* Make sure that no UART transfer is ongoing */
        while (__HAL_UART_GET_FLAG(huart, USART_ISR_BUSY) == SET);

        /* Make sure that UART is ready to receive */
        while (__HAL_UART_GET_FLAG(huart, USART_ISR_REACK) == RESET);

        /* Set the wake-up event: specify wake-up on start-bit detection */
        WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
        HAL_UARTEx_StopModeWakeUpSourceConfig(huart, WakeUpSelection);

        /* Enable the UART Wake UP from stop mode Interrupt */
        __HAL_UART_ENABLE_IT(huart, UART_IT_WUF);

        /* Enable MCU wake-up by UART */
        HAL_UARTEx_EnableStopMode(huart);

        /* Enter STOP mode */
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    } else {
        return BOS_ERROR;
    }
    return BOS_OK;
}

/***************************************************************************/
/* Enable standby mode regarding wake-up pins:
 * WKUP1: PA0 pin
 * WKUP4: PA2 pin
 * WKUP6: PB5 pin
 * WKUP2: PC13 pin
 * NRST pin
 */
BOS_Status EnableStandbyModebyWakeupPinx(WakeupPins_t wakeupPins) {
    /* Clear the WUF flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

    /* Enable the WAKEUP PIN */
    switch (wakeupPins) {
        case PA0_PIN:
            HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
            break;
        case PA2_PIN:
            HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
            break;
        case PB5_PIN:
            HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
            break;
        case PC13_PIN:
            HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
            break;
        case NRST_PIN:
            /* Do nothing */
            break;
    }

    /* Enable SRAM content retention in Standby mode */
    HAL_PWREx_EnableSRAMRetention();

    /* Enter the standby mode */
    HAL_PWR_EnterSTANDBYMode();

    return BOS_OK;
}

/***************************************************************************/
/* Disable standby mode regarding wake-up pins:
 * WKUP1: PA0 pin
 * WKUP4: PA2 pin
 * WKUP6: PB5 pin
 * WKUP2: PC13 pin
 * NRST pin
 */
BOS_Status DisableStandbyModeWakeupPinx(WakeupPins_t wakeupPins) {
    /* Check if the MCU is in standby mode */
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) {
        /* Clear the standby flag */
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

        /* Disable Wake-up Pin */
        switch (wakeupPins) {
            case PA0_PIN:
                HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
                break;
            case PA2_PIN:
                HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
                break;
            case PB5_PIN:
                HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
                break;
            case PC13_PIN:
                HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
                break;
            case NRST_PIN:
                /* Do nothing */
                break;
        }

        /* Blink indicator for 1000ms */
        IND_blink(1000);
    }

    return BOS_OK;
}

/***************************************************************************/
/* Save topology in Read-Only flash memory */
uint8_t SaveTopologyToRO(void) {
    HAL_StatusTypeDef flashStatus = HAL_OK;
    uint16_t flashAdd = 8; /* Starting address offset for topology */
    uint16_t temp = 0;

    /* Unlock the FLASH control register access */
    HAL_FLASH_Unlock();

    /* Erase Topology page */
    FLASH_PageErase(FLASH_BANK_2, TOPOLOGY_PAGE_NUM);

    /* Wait for erase operation to complete */
    flashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
    if (flashStatus != HAL_OK) {
        /* Return FLASH error code */
        return pFlash.ErrorCode;
    } else {
        /* Operation is completed, disable the PER Bit */
        CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
    }

    /* Save module's ID and topology */
    if (myID) {
        /* Save module's ID */
        temp = (uint16_t)(N << 8) + myID;
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, TOPOLOGY_START_ADDRESS, temp);

        /* Wait for write operation to complete */
        flashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
        if (flashStatus != HAL_OK) {
            /* Return FLASH error code */
            return pFlash.ErrorCode;
        } else {
            /* Operation is completed, disable the PG Bit */
            CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
        }

        /* Save topology array */
        for (uint8_t row = 1; row <= N; row++) {
            for (uint8_t column = 0; column <= MAX_NUM_OF_PORTS; column++) {
                /* Check if module serial number exists */
                if (Array[row - 1][0]) {
                    /* Save topology element in flash */
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, TOPOLOGY_START_ADDRESS + flashAdd, Array[row - 1][column]);
                    /* Wait for write operation to complete */
                    flashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
                    if (flashStatus != HAL_OK) {
                        /* Return FLASH error code */
                        return pFlash.ErrorCode;
                    } else {
                        /* Operation is completed, disable the PG Bit */
                        CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
                        /* Update flash address */
                        flashAdd += 8;
                    }
                }
            }
        }
    }

    /* Lock the FLASH control register access */
    HAL_FLASH_Lock();
    return 0;
}

/***************************************************************************/
/* Save command snippets in Read-Only flash memory */
uint8_t SaveSnippetsToRO(void) {
    HAL_StatusTypeDef FlashStatus = HAL_OK;
    uint8_t snipBuffer[sizeof(Snippet_t) + 1] = {0};

    /* Unlock the FLASH control register access */
    HAL_FLASH_Unlock();

    /* Erase Snippets page */
    FLASH_PageErase(FLASH_BANK_2, SNIPPETS_PAGE_NUM);

    /* Wait for erase operation to complete */
    FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
    if (FlashStatus != HAL_OK) {
        /* Return FLASH error code */
        return pFlash.ErrorCode;
    } else {
        /* Operation is completed, disable the PER Bit */
        CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
    }

    /* Save command snippets */
    int currentAdd = SNIPPETS_START_ADDRESS;
    for (uint8_t index = 0; index < NumOfRecordedSnippets; index++) {
        /* Check if snippet condition is valid */
        if (Snippets[index].Condition.ConditionType) {
            /* Set snippet marker */
            snipBuffer[0] = 0xFE;
            memcpy((uint32_t*)&snipBuffer[1], (uint8_t*)&Snippets[index], sizeof(Snippet_t));

            /* Write snippet structure to flash */
            for (uint8_t j = 0; j < (sizeof(Snippet_t) / 4); j++) {
                HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAdd, *(uint64_t*)&snipBuffer[j * 8]);
                FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
                if (FlashStatus != HAL_OK) {
                    /* Return FLASH error code */
                    return pFlash.ErrorCode;
                } else {
                    /* Operation is completed, disable the PG Bit */
                    CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
                    currentAdd += 8;
                }
            }

            /* Write snippet command string to flash */
            for (uint8_t j = 0; j < ((strlen(Snippets[index].CMD) + 1) / 4); j++) {
                HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAdd, *(uint64_t*)(Snippets[index].CMD + j * 4));
                FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
                if (FlashStatus != HAL_OK) {
                    /* Return FLASH error code */
                    return pFlash.ErrorCode;
                } else {
                    /* Operation is completed, disable the PG Bit */
                    CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
                    currentAdd += 8;
                }
            }
        }
    }

    /* Lock the FLASH control register access */
    HAL_FLASH_Lock();
    return 0;
}

/***************************************************************************/
/* Clear topology in SRAM and Flash RO */
uint8_t ClearROtopology(void) {
    /* Clear the topology array */
    memset(Array, 0, sizeof(Array));
    N = 1;
    myID = 0;

    /* Save cleared topology to flash */
    return SaveTopologyToRO();
}

/***************************************************************************/
/* Trigger ST factory bootloader update for a remote module */
void RemoteBootloaderUpdate(uint8_t src, uint8_t dst, uint8_t inport, uint8_t outport) {
    uint8_t myOutport = 0, lastModule = 0;
    int8_t *pcOutputString;

    /* Find route to destination module */
    myOutport = FindRoute(myID, dst);
    if (outport && dst == myID) {
        /* This is a 'via port' update and I'm the last module */
        myOutport = outport;
        lastModule = myID;
    } else if (outport == 0) {
        /* This is a remote update */
        if (NumberOfHops(dst) == 1)
            lastModule = myID;
        else
            lastModule = Route[NumberOfHops(dst) - 1]; /* Previous module */
    }

    /* If this is the source of the message, show status on the CLI */
    if (src == myID) {
        /* Obtain the address of the output buffer */
        pcOutputString = FreeRTOS_CLIGetOutputBuffer();
        if (outport == 0) {
            /* Remote module update */
            sprintf((char*)pcOutputString, pcRemoteBootloaderUpdateMessage, dst);
        } else {
            /* 'Via port' remote update */
            sprintf((char*)pcOutputString, pcRemoteBootloaderUpdateViaPortMessage, dst, outport);
        }
        strcat((char*)pcOutputString, pcRemoteBootloaderUpdateWarningMessage);
        writePxITMutex(inport, (char*)pcOutputString, strlen((char*)pcOutputString), cmd50ms);
        Delay_ms(100);
    }

    /* Setup inport and outport for bootloader update */
    SetupPortForRemoteBootloaderUpdate(inport);
    SetupPortForRemoteBootloaderUpdate(myOutport);

    /* Build a DMA stream between inport and outport */
    StartScastDMAStream(inport, myID, myOutport, myID, BIDIRECTIONAL, 0xFFFFFFFF, 0xFFFFFFFF, false);
}

/***************************************************************************/
/* Setup a port for remote ST factory bootloader update:
 * Set baudrate to 57600
 * Enable even parity
 * Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port) {
    UART_HandleTypeDef *huart = GetUart(port);

    /* Deinitialize UART */
    HAL_UART_DeInit(huart);

    /* Configure UART for bootloader */
    huart->Init.Parity = UART_PARITY_EVEN;
    huart->Init.WordLength = UART_WORDLENGTH_9B;
    HAL_UART_Init(huart);

    /* Ensure RXNE interrupt is enabled */
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

/***************************************************************************/
/* H19R0 module initialization */
void Module_Peripheral_Init(void) {
    /* Initialize UART ports */
	UARTInitSTSPIN();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_USART5_UART_Init();
    MX_USART6_UART_Init();

    /* Configure DMA channels for UARTs */
    for (int i = 1; i <= NUM_OF_PORTS; i++) {
        if (GetUart(i) == &huart1) {
            dmaIndex[i - 1] = &(DMA1_Channel1->CNDTR);
        } else if (GetUart(i) == &huart2) {
            dmaIndex[i - 1] = &(DMA1_Channel2->CNDTR);
        } else if (GetUart(i) == &huart3) {
            dmaIndex[i - 1] = &(DMA1_Channel3->CNDTR);
        } else if (GetUart(i) == &huart4) {
            dmaIndex[i - 1] = &(DMA1_Channel4->CNDTR);
        } else if (GetUart(i) == &huart5) {
            dmaIndex[i - 1] = &(DMA1_Channel5->CNDTR);
        } else if (GetUart(i) == &huart6) {
            dmaIndex[i - 1] = &(DMA1_Channel6->CNDTR);
        }
    }
}

/***************************************************************************/
/* Samples a module parameter value based on parameter index */
Module_Status GetModuleParameter(uint8_t paramIndex, float *value) {
    Module_Status status = BOS_OK;

    /* Check parameter index */
    switch (paramIndex) {
        default:
            /* Invalid parameter index */
            status = BOS_ERR_WrongParam;
            break;
    }

    return status;
}

/***************************************************************************/
/**
 * @brief Handles module messaging tasks for motor control.
 * @param code Message code (e.g., CODE_H19R0_STOP, CODE_H19R0_SET_POSITION).
 * @param port Port number receiving the message.
 * @param src Source module ID.
 * @param dst Destination module ID.
 * @param shift Shift offset for message data.
 * @retval Module_Status Returns H19R0_OK on success, H19R0_ERR_UnknownMessage on unknown code.
 */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift) {
    Module_Status result = H19R0_OK;
    float position = 0.0f, duration = 0.0f;
    uint16_t time = 0;
    int16_t value = 0;
    uint32_t Number_int;

    switch (code) {
        case CODE_H19R0_STOP:
            result = MotorTurnOff();
            break;

        case CODE_H19R0_SET_POSITION:
            /* Extract position */
            Number_int = ((uint32_t)cMessage[port - 1][shift] +
                          ((uint32_t)cMessage[port - 1][1 + shift] << 8) +
                          ((uint32_t)cMessage[port - 1][2 + shift] << 16) +
                          ((uint32_t)cMessage[port - 1][3 + shift] << 24));
            position = *((float*)&Number_int);
            /* Extract duration */
            Number_int = ((uint32_t)cMessage[port - 1][4 + shift] +
                          ((uint32_t)cMessage[port - 1][5 + shift] << 8) +
                          ((uint32_t)cMessage[port - 1][6 + shift] << 16) +
                          ((uint32_t)cMessage[port - 1][7 + shift] << 24));
            duration = *((float*)&Number_int);
            result = MotorMoveToAngle(position, duration);
            break;

        case CODE_H19R0_SET_SPEED:
        case CODE_H19R0_SET_TORQUE:
            time = ((int16_t)cMessage[port - 1][shift]) + ((int16_t)cMessage[port - 1][1 + shift] << 8);
            value = ((int16_t)cMessage[port - 1][2 + shift]) + ((int16_t)cMessage[port - 1][3 + shift] << 8);
            result = (code == CODE_H19R0_SET_SPEED) ? MotorSpeedControl(time, value) : MotorSetTorque(time, value);
            break;

        default:
            result = H19R0_ERR_UNKNOWNMESSAGE;
            break;
    }
    return result;
}
/***************************************************************************/
/* Get the port for a given UART */
uint8_t GetPort(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART6)
		return P1;
	else if (huart->Instance == USART2)
		return P2;
	else if (huart->Instance == USART3)
		return P3;
	else if (huart->Instance == USART5)
		return P4;

	return 0;
}

/***************************************************************************/
/* Register module CLI commands */
void RegisterModuleCLICommands(void) {
	FreeRTOS_CLIRegisterCommand(&CLI_MotorTurnOffCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_MotorMoveToAngleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_MotorSpeedControlCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_MotorSetTorqueCommandDefinition);
}

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
/**
 * @brief Stops the motor.
 * @retval Module_Status Returns H19R0_OK on success.
 */
Module_Status MotorTurnOff(void) {
    Stop(); // Assuming Stop() is defined in a motor control library
    return H19R0_OK;
}
/***************************************************************************/
/**
 * @brief Sets the motor position and duration.
 * @param Position Target position (float).
 * @param Duration Duration to reach the position (float).
 * @retval Module_Status Returns H19R0_OK on success.
 */
Module_Status MotorMoveToAngle(float Position, float Duration) {
    SetPosition(Position *2, Duration); // Assuming SetPosition() is defined in a motor control library
    return H19R0_OK;
}

/***************************************************************************/
/**
 * @brief Sets the motor speed and duration.
 * @param Time Duration in milliseconds.
 * @param Speed Target speed (int16_t).
 * @retval Module_Status Returns H19R0_OK on success.
 */
Module_Status MotorSpeedControl(uint16_t Time, int16_t Speed) {
    SetSpeed(Time, Speed); // Assuming SetSpeed() is defined in a motor control library
    return H19R0_OK;
}

/***************************************************************************/
/**
 * @brief Sets the motor torque and duration.
 * @param Time Duration in milliseconds.
 * @param Torque Target torque (int16_t).
 * @retval Module_Status Returns H19R0_OK on success.
 */
Module_Status MotorSetTorque(uint16_t Time, int16_t Torque) {
    SetTorque(Time, Torque); // Assuming SetTorque() is defined in a motor control library
    return H19R0_OK;
}

/***************************************************************************/
/**
 * @brief Gets the current motor position.
 * @param Position Pointer to store the current position.
 * @retval Module_Status Returns H19R0_OK on success.
 */
Module_Status MotorGetAngle(float *Position) {
    GetPosition(Position); // Assuming GetPosition() is defined in a motor control library
    return H19R0_OK;
}

/***************************************************************************/
/********************************* Commands ********************************/
/***************************************************************************/
/**
 * @brief CLI command to stop the motor.
 * @param pcWriteBuffer Buffer to store the command output.
 * @param xWriteBufferLen Length of the write buffer.
 * @param pcCommandString Command string.
 * @retval portBASE_TYPE Returns pdFALSE to indicate command completion.
 */
portBASE_TYPE CLI_MotorTurnOffCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    Module_Status status = MotorTurnOff();
    strcpy((char *)pcWriteBuffer, (status == H19R0_OK) ? "Motor stopped.\n\r" : "Failed to stop motor.\n\r");
    return pdFALSE;
}
/***************************************************************************/
/**
 * @brief CLI command to set the motor position and duration.
 * @param pcWriteBuffer Buffer to store the command output.
 * @param xWriteBufferLen Length of the write buffer.
 * @param pcCommandString Command string with position and duration parameters.
 * @retval portBASE_TYPE Returns pdFALSE to indicate command completion.
 */
portBASE_TYPE CLI_MotorMoveToAngleCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    portBASE_TYPE len1, len2;
    float position = strtof((char *)FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1), NULL);
    float duration = strtof((char *)FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2), NULL);
    Module_Status status = MotorMoveToAngle(position, duration);
    sprintf((char *)pcWriteBuffer, (status == H19R0_OK) ? "Position: %.2f Duration: %.2f\n\r" : "Error setting position.\n\r", position, duration);
    return pdFALSE;
}
/***************************************************************************/
/**
 * @brief CLI command to set the motor speed and duration.
 * @param pcWriteBuffer Buffer to store the command output.
 * @param xWriteBufferLen Length of the write buffer.
 * @param pcCommandString Command string with time and speed parameters.
 * @retval portBASE_TYPE Returns pdFALSE to indicate command completion.
 */
portBASE_TYPE CLI_MotorSpeedControlCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    portBASE_TYPE len1, len2;
    uint16_t time = (uint16_t)atoi((char *)FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1));
    int16_t speed = (int16_t)atoi((char *)FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2));
    Module_Status status = MotorSpeedControl(time, speed);
    sprintf((char *)pcWriteBuffer, (status == H19R0_OK) ? "Speed: %d Time: %dms\n\r" : "Error setting speed.\n\r", speed, time);
    return pdFALSE;
}
/***************************************************************************/
/**
 * @brief CLI command to set the motor torque and duration.
 * @param pcWriteBuffer Buffer to store the command output.
 * @param xWriteBufferLen Length of the write buffer.
 * @param pcCommandString Command string with time and torque parameters.
 * @retval portBASE_TYPE Returns pdFALSE to indicate command completion.
 */
portBASE_TYPE CLI_MotorSetTorqueCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    portBASE_TYPE len1, len2;
    uint16_t time = (uint16_t)atoi((char *)FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1));
    int16_t torque = (int16_t)atoi((char *)FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2));
    Module_Status status = MotorSetTorque(time, torque);
    sprintf((char *)pcWriteBuffer, (status == H19R0_OK) ? "Torque: %d Time: %dms\n\r" : "Error setting torque.\n\r", torque, time);
    return pdFALSE;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
