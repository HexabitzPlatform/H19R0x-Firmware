/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H19R0.c
 Description   : Source code for module H19R0.
 IR Time-if-Flight (ToF) Sensor (ST VL53L1CX)

 Required MCU resources :

 >> USARTs 1,2,3,4,5,6 for module ports (H19R0).
 >> I2C2 for the ToF sensor.
 >> GPIOB 1 for ToF interrupt (INT).
 >> GPIOA 5 for ToF shutdown (XSHUT).

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include <stdlib.h>


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;
EventGroupHandle_t handleNewReadyData = NULL;
/* variables for Streams ----------------------------------------------------*/
uint32_t numofsamples[2], Timeout[2];
uint8_t Port[2], Module[2], mode[2];
uint8_t bldcMode;
uint16_t Index =0;

/* Module exported parameters ------------------------------------------------*/
module_param_t modParam[NUM_MODULE_PARAMS];

/* Private variables ---------------------------------------------------------*/
TaskHandle_t BLDC_TaskTaskHandle = NULL;
static bool stopStream = false;
uint8_t StopeCliStreamFlag;
/* Exported Typedef ----------------------------------------------------------*/
typedef void (*SampleMemsToBuffer)(float *buffer);
typedef void (*SampleToString)(char*,size_t);

/* Private function prototypes -----------------------------------------------*/
void BLDCTask(void *argument);
Module_Status Exporttoport(uint8_t module, uint8_t port, All_Data function);
Module_Status Exportstreamtoport(uint8_t module,uint8_t port,All_Data function,uint32_t Numofsamples,uint32_t timeout);
Module_Status Exportstreamtoterminal(uint8_t Port,All_Data function,uint32_t Numofsamples,uint32_t timeout);
static Module_Status PollingSleepCLISafe(uint32_t period,long Numofsamples);
static Module_Status StreamMemsToBuf(float *buffer,uint32_t Numofsamples,uint32_t timeout,SampleMemsToBuffer function);
void SamplePosBuff(float *buffer);
/* Create CLI commands --------------------------------------------------------*/
static portBASE_TYPE SampleMotorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE StreamMotorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/* Private CLI functions */
static Module_Status StreamToCLI(uint32_t Numofsamples,uint32_t timeout,SampleToString function);
void SamplePosToString(char *cstring,size_t maxLen);
/*-----------------------------------------------------------*/
/* CLI command structure : sample */
const CLI_Command_Definition_t SampleCommandDefinition = {
	(const int8_t *) "sample",
	(const int8_t *) "sample:\r\n Syntax: sample [Pos]/[Mod].\r\n\r\n",
	SampleMotorCommand,
	1
};

/*-----------------------------------------------------------*/
/* CLI command structure : stream */
const CLI_Command_Definition_t StreamCommandDefinition = {
	(const int8_t *) "stream",
	(const int8_t *) "stream:\r\n Syntax: stream [Pos]/[Mod] ( Numofsamples ) (timeout) .\r\n\r\n",
	StreamMotorCommand,
	-1
};
/*-----------------------------------------------------------*/
/* CLI command structure : streamtcli */
/*-----------------------------------------------------------*/
static Module_Status StreamToCLI(uint32_t Numofsamples,uint32_t timeout,SampleToString function){
	Module_Status status =H19R0_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period =timeout / Numofsamples;
	if(period < MIN_MEMS_PERIOD_MS)
		return H19R0_ERR_WrongParams;

	// TODO: Check if CLI is enable or not
	for(uint8_t chr =0; chr < MSG_RX_BUF_SIZE; chr++){
		if(UARTRxBuf[PcPort - 1][chr] == '\r'){
			UARTRxBuf[PcPort - 1][chr] =0;
		}
	}
	if(1 == StopeCliStreamFlag){
		StopeCliStreamFlag =0;
		static char *pcOKMessage =(int8_t* )"Stop stream !\n\r";
		writePxITMutex(PcPort,pcOKMessage,strlen(pcOKMessage),10);
		return status;
	}
	if(period > timeout)
		timeout =period;

	long numTimes =timeout / period;
	stopStream = false;

	while((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)){
		pcOutputString =FreeRTOS_CLIGetOutputBuffer();
		function((char* )pcOutputString,100);

		writePxMutex(PcPort,(char* )pcOutputString,strlen((char* )pcOutputString),cmd500ms,HAL_MAX_DELAY);
		if(PollingSleepCLISafe(period,Numofsamples) != H19R0_OK)
			break;
	}

	memset((char* )pcOutputString,0,configCOMMAND_INT_MAX_OUTPUT_SIZE);
	sprintf((char* )pcOutputString,"\r\n");
	return status;
}
/*-----------------------------------------------------------*/
void SamplePosToString(char *cstring,size_t maxLen){

	float pose;

	GetPositionMotor(&pose);

	snprintf(cstring,maxLen,"Pos(rad) : %.2f \r\n",pose);

}
/*-----------------------------------------------------------*/
/* CLI command structure : sampletoport */

/* -----------------------------------------------------------------------
 |                        Private Functions                              |
 -----------------------------------------------------------------------
 */

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            PREDIV                         = 1
 *            PLLMUL                         = 6
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 12;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	/** Initializes the peripherals clocks
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC
			| RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;

	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}

/* --- Trigger ST factory bootloader update for a remote module.
 */
void remoteBootloaderUpdate(uint8_t src, uint8_t dst, uint8_t inport,
		uint8_t outport) {

	uint8_t myOutport = 0, lastModule = 0;
	int8_t *pcOutputString;

	/* 1. Get route to destination module */
	myOutport = FindRoute(myID, dst);
	if (outport && dst == myID) { /* This is a 'via port' update and I'm the last module */
		myOutport = outport;
		lastModule = myID;
	} else if (outport == 0) { /* This is a remote update */
		if (NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = route[NumberOfHops(dst)-1]; /* previous module = route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if (src == myID) {
		/* Obtain the address of the output buffer.  Note there is no mutual
		 exclusion on this buffer as it is assumed only one command console
		 interface will be used at any one time. */
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		if (outport == 0)		// This is a remote module update
			sprintf((char*) pcOutputString, pcRemoteBootloaderUpdateMessage,
					dst);
		else
			// This is a 'via port' remote update
			sprintf((char*) pcOutputString,
					pcRemoteBootloaderUpdateViaPortMessage, dst, outport);

		strcat((char*) pcOutputString, pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport, (char*) pcOutputString,
				strlen((char*) pcOutputString), cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);

	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport, myID, myOutport, myID, BIDIRECTIONAL,
			0xFFFFFFFF, 0xFFFFFFFF, false);
}

/* --- Setup a port for remote ST factory bootloader update:
 - Set baudrate to 57600
 - Enable even parity
 - Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port) {
	UART_HandleTypeDef *huart = GetUart(port);

	huart->Init.BaudRate = 57600;
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

/* --- H19R0 module initialization.
 */
void Module_Peripheral_Init(void) {

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	//Circulating DMA Channels ON All Module
	for (int i = 1; i <= NumOfPorts; i++) {
		if (GetUart(i) == &huart1) {
			index_dma[i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			index_dma[i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			index_dma[i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart4) {
			index_dma[i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart5) {
			index_dma[i - 1] = &(DMA1_Channel5->CNDTR);
		} else if (GetUart(i) == &huart6) {
			index_dma[i - 1] = &(DMA1_Channel6->CNDTR);
		}
	}

	/* Create a BLDC task */
	xTaskCreate(BLDCTask, (const char*) "BLDCTask",
			(2 * configMINIMAL_STACK_SIZE), NULL,
			osPriorityNormal - osPriorityIdle, &BLDC_TaskTaskHandle);

}

/*-----------------------------------------------------------*/

/* --- Save array topology and Command Snippets in Flash RO ---
 */
uint8_t SaveToRO(void) {
	BOS_Status result = BOS_OK;
	HAL_StatusTypeDef FlashStatus = HAL_OK;
	uint16_t add = 8;
	uint16_t temp = 0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] = { 0 };

	HAL_FLASH_Unlock();
	/* Erase RO area */
	FLASH_PageErase(FLASH_BANK_1, RO_START_ADDRESS);
	FlashStatus = FLASH_WaitForLastOperation(
			(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	FLASH_PageErase(FLASH_BANK_1, RO_MID_ADDRESS);
	//TOBECHECKED
	FlashStatus = FLASH_WaitForLastOperation(
			(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	if (FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	} else {
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}

	/* Save number of modules and myID */
	if (myID) {
		temp = (uint16_t) (N << 8) + myID;
		//HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,RO_START_ADDRESS,temp);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, RO_START_ADDRESS, temp);
		//TOBECHECKED
		FlashStatus = FLASH_WaitForLastOperation(
				(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
		if (FlashStatus != HAL_OK) {
			return pFlash.ErrorCode;
		} else {
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
		}

		/* Save topology */
		for (uint8_t i = 1; i <= N; i++) {
			for (uint8_t j = 0; j <= MaxNumOfPorts; j++) {
				if (array[i - 1][0]) {

					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
							RO_START_ADDRESS + add, array[i - 1][j]);
					//HALFWORD 	//TOBECHECKED
					FlashStatus = FLASH_WaitForLastOperation(
							(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
					if (FlashStatus != HAL_OK) {
						return pFlash.ErrorCode;
					} else {
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
						add += 8;
					}
				}
			}
		}
	}

	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for (uint8_t s = 0; s < numOfRecordedSnippets; s++) {
		if (snippets[s].cond.conditionType) {
			snipBuffer[0] = 0xFE;		// A marker to separate Snippets
			memcpy((uint32_t*) &snipBuffer[1], (uint8_t*) &snippets[s],
					sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for (uint8_t j = 0; j < (sizeof(snippet_t) / 4); j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAdd,
						*(uint64_t*) &snipBuffer[j * 8]);
				//HALFWORD
				//TOBECHECKED
				FlashStatus = FLASH_WaitForLastOperation(
						(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 8;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for (uint8_t j = 0; j < ((strlen(snippets[s].cmd) + 1) / 4); j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAdd,
						*(uint64_t*) (snippets[s].cmd + j * 4));
				//HALFWORD
				//TOBECHECKED
				FlashStatus = FLASH_WaitForLastOperation(
						(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 8;
				}
			}
		}
	}

	HAL_FLASH_Lock();

	return result;
}

/* --- Clear array topology in SRAM and Flash RO --- 
 */
uint8_t ClearROtopology(void) {
	// Clear the array
	memset(array, 0, sizeof(array));
	N = 1;
	myID = 0;

	return SaveToRO();
}

/* --- H19R0 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift)
{
  Module_Status result = H19R0_OK;

  switch (code)
  {

	default:
		result = H19R0_ERR_UnknownMessage;
		break;
  }

  return result;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void) {
	FreeRTOS_CLIRegisterCommand(&SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&StreamCommandDefinition);
}

/*-----------------------------------------------------------*/

/* --- Get the port for a given UART.
 */
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

/*-----------------------------------------------------------*/

/* --- BLDC streaming task
 */

void BLDCTask(void *argument) {

	/* Infinite loop */
	for (;;) {
		switch (bldcMode) {
		case STREAM_TO_PORT:
			Exportstreamtoport(Module[0], Port[0], mode[0], numofsamples[0],
					Timeout[0]);
			break;
		case STREAM_TO_Terminal:
			Exportstreamtoterminal(Port[1], mode[1], numofsamples[1],
					Timeout[1]);
			break;
		default:
			osDelay(10);
			break;
		}

		taskYIELD();
	}
}

/*-----------------------------------------------------------*/
Module_Status Exporttoport(uint8_t module, uint8_t port, All_Data function) {

	float position;
	static uint8_t temp[4] = { 0 };
	Module_Status status = H19R0_OK;

	switch (function) {
	case POS:

		if ((status = GetPositionMotor(&position)) != H19R0_OK)
			return status = H19R0_ERROR;

		if (module == myID || module == 0) {
			temp[0] =(uint8_t )((*(uint32_t* )&position) >> 0);
			temp[1] =(uint8_t )((*(uint32_t* )&position) >> 8);
			temp[2] =(uint8_t )((*(uint32_t* )&position) >> 16);
			temp[3] =(uint8_t )((*(uint32_t* )&position) >> 24);

			writePxITMutex(port, (char*) &temp[0], 4* sizeof(uint8_t), 10);
		}
		break;
	default:
		status = H19R0_ERR_WrongParams;
		break;
	}


	return status;
}

/*-----------------------------------------------------------*/
Module_Status Exportstreamtoport(uint8_t module,uint8_t port,All_Data function,uint32_t Numofsamples,uint32_t timeout){
	Module_Status status =H19R0_OK;
	uint32_t samples =0;
	uint32_t period =0;
	period =timeout / Numofsamples;

	if(timeout < MIN_PERIOD_MS || period < MIN_PERIOD_MS)
		return H19R0_ERR_WrongParams;

	while(samples < Numofsamples){
		status =Exporttoport(module,port,function);
		vTaskDelay(pdMS_TO_TICKS(period));
		samples++;
	}
	bldcMode = DEFAULT;

	return status;
}
/*-----------------------------------------------------------*/
Module_Status Exportstreamtoterminal(uint8_t Port,All_Data function,uint32_t Numofsamples,uint32_t timeout){
	Module_Status status =H19R0_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period =timeout / Numofsamples;
	char cstring[100];
	float position =0;

	if(period < MIN_MEMS_PERIOD_MS)
		return H19R0_ERR_WrongParams;

	// TODO: Check if CLI is enable or not
	switch(function){
		case POS:

			if(period > timeout)
				timeout =period;

			stopStream = false;

			while((Numofsamples-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)){
				pcOutputString =FreeRTOS_CLIGetOutputBuffer();
				if((status =GetPositionMotor(&position)) != H19R0_OK)
					return status;

				snprintf(cstring,50,"Position(rad) : %.2f \r\n",position);

				writePxMutex(Port,(char* )cstring,strlen((char* )cstring),
				cmd500ms,HAL_MAX_DELAY);
				if(PollingSleepCLISafe(period,Numofsamples) != H19R0_OK)
					break;
			}
			break;

		default:
			status =H19R0_ERR_WrongParams;
			break;
	}

	bldcMode = DEFAULT;
	return status;
}
/*-----------------------------------------------------------*/
static Module_Status PollingSleepCLISafe(uint32_t period,long Numofsamples){
	const unsigned DELTA_SLEEP_MS =100; // milliseconds
	long numDeltaDelay =period / DELTA_SLEEP_MS;
	unsigned lastDelayMS =period % DELTA_SLEEP_MS;

	while(numDeltaDelay-- > 0){
		vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

		// Look for ENTER key to stop the stream
		for(uint8_t chr =1; chr < MSG_RX_BUF_SIZE; chr++){
			if(UARTRxBuf[PcPort - 1][chr] == '\r'){
				UARTRxBuf[PcPort - 1][chr] =0;
				StopeCliStreamFlag =1;
				return H19R0_ERR_TERMINATED;
			}
		}

		if(stopStream)
			return H19R0_ERR_TERMINATED;
	}

	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));
	return H19R0_OK;
}

/*-----------------------------------------------------------*/
static Module_Status StreamMemsToBuf(float *buffer,uint32_t Numofsamples,uint32_t timeout,SampleMemsToBuffer function){
	Module_Status status =H19R0_OK;
	uint32_t period =timeout / Numofsamples;
	if(period < MIN_MEMS_PERIOD_MS)
		return H19R0_ERR_WrongParams;

	// TODO: Check if CLI is enable or not

	if(period > timeout)
		timeout =period;

	long numTimes =timeout / period;
	stopStream = false;

	while((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)){
		if(function == SamplePosBuff){
			float sample;
			function(&sample);
			buffer[Index] =sample;
			Index++;

		}

		vTaskDelay(pdMS_TO_TICKS(period));
		if(stopStream){
			status =H19R0_ERR_TERMINATED;
			break;
		}
	}
	return status;
}

/*-----------------------------------------------------------*/
void SamplePosBuff(float *buffer){
	float Pos;
	GetPositionMotor(&Pos);
	*buffer =Pos;
}

/* -----------------------------------------------------------------------
 |                               APIs                                    |
 -----------------------------------------------------------------------
 */

uint8_t SetPositionMotor(float Position, float Duration) {

	SetPosition(Position, Duration);

	return 0;
}


uint8_t GetPositionMotor(float *Position){
	GetPosition(Position);

	return 0;
}

/*-----------------------------------------------------------*/
Module_Status SampletoPort(uint8_t module,uint8_t port,All_Data function){
	Module_Status status =H19R0_OK;

	if(port == 0 && module == myID)
		return status =H19R0_ERR_WrongParams;

	Exporttoport(module,port,function);

	return status;

}
/*-----------------------------------------------------------*/
Module_Status StreamtoPort(uint8_t module,uint8_t port,All_Data function,uint32_t Numofsamples,uint32_t timeout){
	Module_Status status =H19R0_OK;

	if(port == 0 && module == myID)
		return status =H19R0_ERR_WrongParams;

	bldcMode =STREAM_TO_PORT;
	Port[0] =port;
	Module[0] =module;
	numofsamples[0] =Numofsamples;
	Timeout[0] =timeout;
	mode[0] =function;
	return status;
}

/*-----------------------------------------------------------*/

Module_Status StreamToTerminal(uint8_t port,All_Data function,uint32_t Numofsamples,uint32_t timeout){
	Module_Status status =H19R0_OK;

	if(0 == port)
		return status =H19R0_ERR_WrongParams;

	bldcMode =STREAM_TO_Terminal;
	Port[1] =port;
	numofsamples[1] =Numofsamples;
	Timeout[1] =timeout;
	mode[1] =function;
	return status;
}

/*-----------------------------------------------------------*/

Module_Status StreamToBuffer(float *buffer,All_Data function,uint32_t Numofsamples,uint32_t timeout){

	switch(function){
		case POS:
			return StreamMemsToBuf(buffer,Numofsamples,timeout,SamplePosBuff);
			break;
		default:
			break;
	}
}


/* -----------------------------------------------------------------------
 |                             Commands                                  |
 -----------------------------------------------------------------------
 */
static portBASE_TYPE SampleMotorCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	const char *const PosCmdName ="pos";


	const char *pSensName = NULL;
	portBASE_TYPE sensNameLen =0;

	// Make sure we return something
	*pcWriteBuffer ='\0';

	pSensName =(const char* )FreeRTOS_CLIGetParameter(pcCommandString,1,&sensNameLen);

	if(pSensName == NULL){
		snprintf((char* )pcWriteBuffer,xWriteBufferLen,"Invalid Arguments\r\n");
		return pdFALSE;
	}

	do{
		if(!strncmp(pSensName,PosCmdName,strlen(PosCmdName))){
			Exportstreamtoterminal(PcPort,POS,1,500);

		}
		else{
			snprintf((char* )pcWriteBuffer,xWriteBufferLen,"Invalid Arguments\r\n");
		}

		return pdFALSE;
	} while(0);

	snprintf((char* )pcWriteBuffer,xWriteBufferLen,"Error reading Sensor\r\n");
	return pdFALSE;
}

/*-----------------------------------------------------------*/
// Port Mode => false and CLI Mode => true
static bool StreamCommandParser(const int8_t *pcCommandString,const char **ppSensName,portBASE_TYPE *pSensNameLen,
bool *pPortOrCLI,uint32_t *pPeriod,uint32_t *pTimeout,uint8_t *pPort,uint8_t *pModule){
	const char *pPeriodMSStr = NULL;
	const char *pTimeoutMSStr = NULL;

	portBASE_TYPE periodStrLen =0;
	portBASE_TYPE timeoutStrLen =0;

	const char *pPortStr = NULL;
	const char *pModStr = NULL;

	portBASE_TYPE portStrLen =0;
	portBASE_TYPE modStrLen =0;

	*ppSensName =(const char* )FreeRTOS_CLIGetParameter(pcCommandString,1,pSensNameLen);
	pPeriodMSStr =(const char* )FreeRTOS_CLIGetParameter(pcCommandString,2,&periodStrLen);
	pTimeoutMSStr =(const char* )FreeRTOS_CLIGetParameter(pcCommandString,3,&timeoutStrLen);

	// At least 3 Parameters are required!
	if((*ppSensName == NULL) || (pPeriodMSStr == NULL) || (pTimeoutMSStr == NULL))
		return false;

	// TODO: Check if Period and Timeout are integers or not!
	*pPeriod =atoi(pPeriodMSStr);
	*pTimeout =atoi(pTimeoutMSStr);
	*pPortOrCLI = true;

	pPortStr =(const char* )FreeRTOS_CLIGetParameter(pcCommandString,4,&portStrLen);
	pModStr =(const char* )FreeRTOS_CLIGetParameter(pcCommandString,5,&modStrLen);

	if((pModStr == NULL) && (pPortStr == NULL))
		return true;
	if((pModStr == NULL) || (pPortStr == NULL))	// If user has provided 4 Arguments.
		return false;

	*pPort =atoi(pPortStr);
	*pModule =atoi(pModStr);
	*pPortOrCLI = false;

	return true;
}

/*-----------------------------------------------------------*/
static portBASE_TYPE StreamMotorCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	const char *const PosCmdName ="pos";

	uint32_t Numofsamples =0;
	uint32_t timeout =0;
	uint8_t port =0;
	uint8_t module =0;

	bool portOrCLI = true; // Port Mode => false and CLI Mode => true

	const char *pSensName = NULL;
	portBASE_TYPE sensNameLen =0;

	// Make sure we return something
	*pcWriteBuffer ='\0';

	if(!StreamCommandParser(pcCommandString,&pSensName,&sensNameLen,&portOrCLI,&Numofsamples,&timeout,&port,&module)){
		snprintf((char* )pcWriteBuffer,xWriteBufferLen,"Invalid Arguments\r\n");
		return pdFALSE;
	}

	do{
		if(!strncmp(pSensName,PosCmdName,strlen(PosCmdName))){
			if(portOrCLI){
				StreamToCLI(Numofsamples,timeout,SamplePosToString);
			}

		}
		else{
			snprintf((char* )pcWriteBuffer,xWriteBufferLen,"Invalid Arguments\r\n");
		}

		snprintf((char* )pcWriteBuffer,xWriteBufferLen,"\r\n");
		return pdFALSE;
	} while(0);

	snprintf((char* )pcWriteBuffer,xWriteBufferLen,"Error reading Sensor\r\n");
	return pdFALSE;
}



/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
