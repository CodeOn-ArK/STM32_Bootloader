/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
 * All the supported commands are packed here
 */
uint8_t supported_cmnds[] = {
		BL_GET_VER,
		BL_GET_HELP,
		BL_GET_RDP_STATUS,
		BL_GET_CID,
		BL_GO_TO_ADDR,
		BL_FLASH_ERASE,
		BL_MEM_WRITE,
		BL_EN_R_W_PROTECT,
		BL_MEM_READ,
		BL_MEM_WRITE
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEBUG_SUPPORT
#define D_UART (&huart3)
#define C_UART (&huart2)

//#define BL_ACK	0xA5
//#define BL_NACK	0x5A

#define BL_Rx_LEN 200
uint8_t bl_rx_buffer[BL_Rx_LEN];

// Uncomment below line to enable formated printf support

/* USER CODE END PM */


/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

void  bootloader_uart_read_data();
void  bootloader_jump_to_user_app();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void printMsg(char *format,...);

static void printMsg(char *format,...)
{
#ifdef DEBUG_SUPPORT
	char str[80];

	/*extract the arg list using VA APIs */
	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);

	HAL_UART_Transmit(D_UART, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

	va_end(args);
#endif //DEBUG_SUPPORT
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  //SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  if( (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) == GPIO_PIN_RESET)
  {
	  printMsg("\nBOOTLOADER JUMPING TO READ DATA\n\n\r");
	  bootloader_uart_read_data();

  }else{

	  printMsg("\nBOOTLOADER JUMPING TO USER APPLICATION\n\n\r");
	  bootloader_jump_to_user_app();
  }


}

void  bootloader_uart_read_data()
{

	uint8_t rcv_len = 0;
	while(1)
	{
		memset(bl_rx_buffer,0,200);

		//Here we will read and decode the commands coming from host
		//First Read Only one byte from the host, which is the "length" field of the command
		HAL_UART_Receive(C_UART, bl_rx_buffer, 1, HAL_MAX_DELAY);
		rcv_len = bl_rx_buffer[0];

		HAL_UART_Receive(C_UART, &bl_rx_buffer[1], rcv_len, HAL_MAX_DELAY);

		MX_CRC_Init();

		switch(bl_rx_buffer[1])
		{
		case BL_GET_VER :Bootloader_GET_VER(bl_rx_buffer);
						 break;

		case BL_GET_HELP:Bootloader_GET_HELP(bl_rx_buffer);
						 break;

		case BL_GET_CID:Bootloader_GET_CID(bl_rx_buffer);
						 break;

		case BL_GET_RDP_STATUS:Bootloader_GET_RDP_STATUS(bl_rx_buffer);
						 break;

		case BL_GO_TO_ADDR:Bootloader_GO_TO_ADDR(bl_rx_buffer);
						 break;

		case BL_FLASH_ERASE:Bootloader_FLASH_ERASE(bl_rx_buffer);
						 break;

		case BL_MEM_WRITE:Bootloader_MEM_WRITE(bl_rx_buffer);
						 break;

		case BL_EN_R_W_PROTECT:Bootloader_EN_R_W_PROTECT(bl_rx_buffer);
						 break;

		case BL_MEM_READ:Bootloader_MEM_READ(bl_rx_buffer);
						 break;

		case BL_READ_SECTOR_STATUS:Bootloader_READ_SECTOR_STATUS(bl_rx_buffer);
						 break;

		case BL_OTP_READ:Bootloader_OTP_READ(bl_rx_buffer);
						 break;

		case BL_DIS_R_W_PROTECT:Bootloader_DIS_R_W_PROTECT(bl_rx_buffer);
						 break;

		default : printMsg("BL_UART_READ : received invalid code");
		}
	}
}

/*
 * This code jumps to user application if the user button
 * is not pressed after reseting the board
 * Here we jump to FLASH_SECTOR2_BASE_ADDRESS as the Base Address
 * for the USER app
 */
void  bootloader_jump_to_user_app()
{
	//Function ptr to hold the address of the reset handler of the USER application
	void (*app_reset_handler)(void);
	printMsg("BL_DEBUG_MSG : Bootloader_jump_to_user_app\n\r");

	//1. Here we configure the MSP by reading the value stored at FLASH_SECTOR2_BASE_ADDRESS
	volatile uint32_t MSP_VALUE = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
	printMsg("BL_DEBUG_MSG : MSP value : %#x\n\r", MSP_VALUE);

	//2. Now we set the MSP to the value pointed to by FLASH_SECTOR2_BASE_ADDRESS using a CMSIS API
	__set_MSP(MSP_VALUE);

	/*
	 * 3. Now we fetch the RESET HANDLER fo the USER application
	 * from the location (FLASH_SECTOR2_BASE_ADDRESS + 4)
	 */
	volatile uint32_t RESET_HANDLER = *(uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS + 4);
 	printMsg("BL_DEBUG_MSG : RESET HANDLER = %#x \n\r", RESET_HANDLER);

	app_reset_handler = (void *)RESET_HANDLER;   //(void *)*(uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS + 4);  If other things don't work use this twerk

	//4. Now we jump to the USER application by calling the USER's RESET HANDLER
	//which in turn calls the USER application
	app_reset_handler();

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/***************************** Implementation of the Bootloader specified function calls ****************************************/

void Bootloader_GET_VER(uint8_t *pRxBuffer)
{
	uint8_t bl_version;

	//1. verify the checksum
	printMsg("BL_DEBUG_MSG : bootloader_handle_getver_cmd\n\r");

	//2.Total command packet length
	uint32_t packt_len = pRxBuffer[0] + 1;

	//3. Shell out CRC from host from the received buffer
	uint32_t Host_CRC = *((uint32_t *)(pRxBuffer + packt_len - 4));

	if((bootloader_verify_crc(pRxBuffer, packt_len - 4, Host_CRC)) == CRC_VERIFY_SUCCESS)
	{
		printMsg("BL_DEBUG_MSG : checksum success !!\n\r");

		//checksum is correct
		bootloader_send_ack(pRxBuffer[0], 1);
		bl_version = get_bootloader_version();

		printMsg("BL_DEBUG_MSG : BL_VER : %d %#x\n\r", bl_version, bl_version);
		bootloader_uart_write_data(&bl_version,1);

	}else{
		printMsg("BL_DEBUG_MSG : checksum fail !!\n\r");

		//checksum is wrong send nack
		bootloader_send_nack();
	}
}

void Bootloader_GET_HELP(uint8_t *pRxBuffer)
{
	//Send the number of commands supported by the bootloader

	printMsg("BL_DEBUG_MSG : bootloader_get_help\n\r");

	//1. Extract packet length
	uint32_t packt_len = pRxBuffer[0] + 1;

	//2. Extract the CRC sent by HOST
	uint32_t Host_CRC = *((uint32_t *)(pRxBuffer + packt_len - 4));

	//3. Verify CRC
	if(bootloader_verify_crc(pRxBuffer, packt_len - 4, Host_CRC) == CRC_VERIFY_SUCCESS)
	{
		//Checksum is correct
		printMsg("BL_DEBUG_MSG : checksum success !!\n\r");

		//Send ACK to HOST program
		bootloader_send_ack(*pRxBuffer, 10);

		bootloader_uart_write_data(supported_cmnds, sizeof(supported_cmnds));
	}else
	{
		//Checksum failure
		printMsg("BL_DEBUG_MSG : checksum fail !!\n\r");

		//Send NACK to HOST
		bootloader_send_nack();
	}

}

void Bootloader_GET_CID(uint8_t *pRxBuffer)
{
	//Send the number of commands supported by the bootloader

	printMsg("BL_DEBUG_MSG : bootloader_get_CID\n\r");

	//1. Extract packet length
	uint32_t packt_len = pRxBuffer[0] + 1;

	//2. Extract the CRC sent by HOST
	uint32_t Host_CRC = *((uint32_t *)(pRxBuffer + packt_len - 4));

	//3. Verify CRC
	if(bootloader_verify_crc(pRxBuffer, packt_len - 4, Host_CRC) == CRC_VERIFY_SUCCESS)
	{
		//Checksum is correct
		printMsg("BL_DEBUG_MSG : checksum success !!\n\r");

		//Send ACK to HOST program
		bootloader_send_ack(*pRxBuffer, 2);

		//Get the CID
		volatile uint32_t CID = get_CID_info();

		printMsg("BL_DEBUG_MSG : CID_Number : %ld %#x\n\r", CID, CID);

		//send to the host command
		bootloader_uart_write_data(&CID, 2);

	}else
	{
		//Checksum failure
		printMsg("BL_DEBUG_MSG : checksum fail !!\n\r");

		//Send NACK to HOST
		bootloader_send_nack();
	}

}
void Bootloader_GET_RDP_STATUS(uint8_t *pRxBuffer)
{
	//Send the number of commands supported by the bootloader

	printMsg("BL_DEBUG_MSG : bootloader_get_RDP_status\n\r");

	//1. Extract packet length
	uint32_t packt_len = pRxBuffer[0] + 1;

	//2. Extract the CRC sent by HOST
	uint32_t Host_CRC = *((uint32_t *)(pRxBuffer + packt_len - 4));

	//3. Verify CRC
	if(bootloader_verify_crc(pRxBuffer, packt_len - 4, Host_CRC) == CRC_VERIFY_SUCCESS)
	{
		//Checksum is correct
		printMsg("BL_DEBUG_MSG : checksum success !!\n\r");

		//Send ACK to HOST program
		bootloader_send_ack(*pRxBuffer, 1);

		//Get the CID
		volatile uint8_t RDP_Status = get_RDP_info();

		printMsg("BL_DEBUG_MSG : RDP_Number : %ld %#x\n\r", RDP_Status, RDP_Status);

		//send to the host command
		bootloader_uart_write_data(&RDP_Status, 1);

	}else
	{
		//Checksum failure
		printMsg("BL_DEBUG_MSG : checksum fail !!\n\r");

		//Send NACK to HOST
		bootloader_send_nack();
	}

}


void Bootloader_GO_TO_ADDR(uint8_t *pRxBuffer)
{
	//Send the number of commands supported by the bootloader
	printMsg("BL_DEBUG_MSG : bootloader_go_to_addr\n\r");

	//1. Extract packet length
	uint32_t packt_len = pRxBuffer[0] + 1;

	//2. Extract the CRC sent by HOST
	uint32_t Host_CRC = *((uint32_t *)(pRxBuffer + packt_len - 4));

	//3. Verify CRC
	if(bootloader_verify_crc(pRxBuffer, packt_len - 4, Host_CRC) == CRC_VERIFY_SUCCESS)
	{
		//Checksum is correct
		printMsg("BL_DEBUG_MSG : checksum success !!\n\r");

		//Send ACK to HOST program
		bootloader_send_ack(*pRxBuffer, 1);

		//shell out the address sent by HOST
		volatile uint32_t goto_addr = *((uint32_t*)(pRxBuffer + 2));

		printMsg("BL_DEBUG_MSG : Jump address %#x\n\r", goto_addr);

		//Check address validity; jump to address if address valid
		if(!check_validity(goto_addr))
		{
			uint8_t validity = (uint8_t)ADDRESS_VALID;
			bootloader_uart_write_data(&validity, 1);

			//address is valid
			printMsg("BL_DEBUG_MSG : Address valid !!\n\r");

			goto_addr += 1; //T-bit made 1 here

			void (*jump_to_addr)(void) = (void *)goto_addr;


			//jumping to address
			printMsg("BL_DEBUG_MSG : jumping to address\n\r");
			jump_to_addr();

		}else
		{
			uint8_t validity = (uint8_t)ADDRESS_INVALID;
			bootloader_uart_write_data(&validity, 1);

			//address invalid
			printMsg("BL_DEBUG_MSG : Address invalid!!\n\r");
		}

	}else
	{
		//Checksum failure
		printMsg("BL_DEBUG_MSG : checksum fail !!\n\r");

		//Send NACK to HOST
		bootloader_send_nack();
	}

}
void Bootloader_FLASH_ERASE(uint8_t *pRxBuffer)
{
	//1. verify the checksum
	printMsg("BL_DEBUG_MSG : bootloader_FLASH_ERASE\n\r");

	//2.Total command packet length
	uint32_t packt_len = pRxBuffer[0] + 1;

	//3. Shell out CRC from host from the received buffer
	uint32_t Host_CRC = *((uint32_t *)(pRxBuffer + packt_len - 4));

	if((bootloader_verify_crc(pRxBuffer, packt_len - 4, Host_CRC)) == CRC_VERIFY_SUCCESS)
	{
		printMsg("BL_DEBUG_MSG : checksum success !!\n\r");

		//checksum is correct
		bootloader_send_ack(pRxBuffer[0], 1);

		FLASH_EraseInitTypeDef pEraseInit;
		uint32_t SectorError;

		if(*(pRxBuffer+2) != 0xFF)
		{
			pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
			pEraseInit.Sector	 = pRxBuffer[2];
			pEraseInit.NbSectors = pRxBuffer[3];
			pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		}else
		{
			pEraseInit.TypeErase = FLASH_TYPEERASE_MASSERASE;
			pEraseInit.Sector	 = 0;
			pEraseInit.NbSectors = 7;
			pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		}

		HAL_FLASH_Unlock();

		if(HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK) Error_Handler();

		pRxBuffer[0] = 0x0;
		if(SectorError == 0xFFFFFFFF) bootloader_uart_write_data(pRxBuffer, 1);

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	}else{
		printMsg("BL_DEBUG_MSG : checksum fail !!\n\r");

		//checksum is wrong send nack
		bootloader_send_nack();
	}

}
void Bootloader_MEM_WRITE(uint8_t *pRxBuffer)
{

}

void Bootloader_EN_R_W_PROTECT(uint8_t *pRxBuffer)
{

	//Send the number of commands supported by the bootloader

	printMsg("BL_DEBUG_MSG : Bootloader_EN_R_W_PROTECT\n\r");

	//1. Extract packet length
	uint32_t packt_len = pRxBuffer[0] + 1;

	//2. Extract the CRC sent by HOST
	uint32_t Host_CRC = *((uint32_t *)(pRxBuffer + packt_len - 4));

	//3. Verify CRC
	if(bootloader_verify_crc(pRxBuffer, packt_len - 4, Host_CRC) == CRC_VERIFY_SUCCESS)
	{
		//Checksum is correct
		printMsg("BL_DEBUG_MSG : checksum success !!\n\r");

		//Send ACK to HOST program
		bootloader_send_ack(*pRxBuffer, 1);

		//Enable R/W protection of the FLASH area (program OPTION bytes to 0xFF)
		EN_R_W_Protect();

		//printMsg("BL_DEBUG_MSG : RDP_Number : %ld %#x\n\r", RDP_Status, RDP_Status);

		//send to the host command
		//bootloader_uart_write_data(, 1);

	}else
	{
		//Checksum failure
		printMsg("BL_DEBUG_MSG : checksum fail !!\n\r");

		//Send NACK to HOST
		bootloader_send_nack();
	}



}

void Bootloader_DIS_R_W_PROTECT(uint8_t *pRxBuffer)
{

}

void Bootloader_MEM_READ(uint8_t *pRxBuffer)
{

}
void Bootloader_READ_SECTOR_STATUS(uint8_t *pRxBuffer)
{

}
void Bootloader_OTP_READ(uint8_t *pRxBuffer)
{

}



/***********************************************************************************/




void bootloader_send_ack(uint8_t cmnd_code, uint8_t follow_len)
{
	//here we send 2 bytes.. first byte is ack and the second byte is len value
	uint8_t ack_buf[2];

	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;

	HAL_UART_Transmit(C_UART, ack_buf, 2, HAL_MAX_DELAY);
}

void bootloader_send_nack(void)
{
	//here we send nack in response to data invalidity from CRC
	uint8_t NACK[1];
	NACK[0] = (uint8_t)BL_NACK;
	HAL_UART_Transmit(C_UART, NACK, 1, HAL_MAX_DELAY);
}

uint8_t bootloader_verify_crc(uint8_t pData[], uint32_t len, uint32_t crc_host)
{

	volatile uint32_t CRCVal = 0x00000000;
	uint32_t accumulated_data = 0;

	for(uint8_t i=0; i<len; i++)
	{
		accumulated_data = pData[i];
		CRCVal = HAL_CRC_Accumulate(&hcrc, &accumulated_data, 1);
	}

	if(CRCVal == crc_host)
	{
		return CRC_VERIFY_SUCCESS;
	}

		return CRC_VERIFY_FAILURE;

}

uint8_t get_bootloader_version(void)
{
	return BL_VERSION;
}

//This function is called to Send data to the HOST over the C_UART
void bootloader_uart_write_data(uint8_t *pRxBuffer, uint8_t len)
{
	HAL_UART_Transmit(C_UART, pRxBuffer, len, HAL_MAX_DELAY);
}


uint32_t get_CID_info()
{
	return (*DBGMCU_IDCODE);
}

//This function returns Read Protection Level of the OPTION bytes(16B)
uint8_t get_RDP_info()
{
	uint32_t RDP_status = *OPTION_BYTES_BASE_ADDRESS;

	return(RDP_status >> 8);
}


void EN_R_W_Protect(void)
{

}

uint8_t check_validity(uint32_t addr)
{
	/*
	 * valid addresses :-
	 *
	 * FLASH
	 * SRAM1 && SRAM2
	 * RAM
	 * BACKUP SRAM
	 * External FLASH
	 */

	if(addr >= FLASH_BASE && addr <= FLASH_END)
	{
		return ADDRESS_VALID;

	}else if(addr >= SRAM1_BASE && addr <= SRAM1_END)
	{
		return ADDRESS_VALID;

	}else if(addr >= SRAM2_BASE && addr <= SRAM2_END)
	{
		return ADDRESS_VALID;

	}else if(addr >= BKPSRAM_BASE && addr <= BKPSRAM_END)
	{
		return ADDRESS_VALID;

	}

	return ADDRESS_INVALID;

}













/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
