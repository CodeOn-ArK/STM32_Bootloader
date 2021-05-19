/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdarg.h"
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define BL_ACK	0xA5
#define BL_NACK	0x5A
#define CRC_VERIFY_FAILURE 0xAF
#define CRC_VERIFY_SUCCESS 0xFA

//version 1.0
#define BL_VERSION 0x10
/********************************** Macros to define the Bootloader specific commands *****************************************/

/*
 * This Macro is used to read the bootloader version from the MCU
 */
 #define BL_GET_VER 0x51

/*
 * This Macro is used to know what are the supported commands
 */
 #define BL_GET_HELP 0x52

/*
 * This Macro is used to read the MCU Chip Identification Number
 */
 #define BL_GET_CID 0x53

/*
 * This Macro is used to read FLASH Read Protection Level
 */
 #define BL_GET_RDP_STATUS 0x54

/*
 * This Macro is used to jump to specified address
 */
 #define BL_GO_TO_ADDR 0x55

/*
 * This Macro is used to mass erase or sector erase of the USER FLASH
 */
 #define BL_FLASH_ERASE 0x56

/*
 * This Macro is used to write data into different memories of the MCU
 */
 #define BL_MEM_WRITE 0x57

/*
 * This Macro is used to enable read/write protection on different sectors of the user FLSH
 */
 #define BL_EN_R_W_PROTECT 0x58

/*
 * This Macro is used to read data from different memories of th uC
 */
 #define BL_MEM_READ 0x59

/*
 * This Macro is used to read all the sector protection status
 */
 #define BL_READ_SECTOR_STATUS 0x5A

/*
 * This Macro is used to read the OTP contents
 */
 #define BL_OTP_READ 0x5B

/*
 * This Macro is used to disable read/write protection on different sectors of the USER FLASH
 */
 #define BL_DIS_R_W_PROTECT 0x5C



/* USER CODE END EM */

//Bootlader specific function prototypes
void  bootloader_uart_read_data();
void  booloader_jump_to_user_app();

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void Bootloader_GET_VER(uint8_t *pRxBuffer);
void Bootloader_GET_HELP(uint8_t *pRxBuffer);
void Bootloader_GET_CID(uint8_t *pRxBuffer);
void Bootloader_GET_RDP_STATUS(uint8_t *pRxBuffer);
void Bootloader_GO_TO_ADDR(uint8_t *pRxBuffer);
void Bootloader_FLASH_ERASE(uint8_t *pRxBuffer);
void Bootloader_MEM_WRITE(uint8_t *pRxBuffer);
void Bootloader_EN_R_W_PROTECT(uint8_t *pRxBuffer);
void Bootloader_MEM_READ(uint8_t *pRxBuffer);
void Bootloader_READ_SECTOR_STATUS(uint8_t *pRxBuffer);
void Bootloader_OTP_READ(uint8_t *pRxBuffer);
void Bootloader_DIS_R_W_PROTECT(uint8_t *pRxBuffer);

void bootloader_send_nack(void);
void bootloader_send_ack(uint8_t cmnd_code, uint8_t follow_len);
uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host);
uint8_t get_bootloader_version();
void bootloader_uart_write_data(uint8_t *pRxBuffer, uint8_t len);
uint32_t get_CID_info();
uint8_t get_RDP_info();
void EN_R_W_Protect(void);
uint8_t check_validity(uint32_t);
uint32_t flash_eraser(uint8_t sector_num, uint8_t num_of_sector);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin 			GPIO_PIN_13
#define B1_GPIO_Port 	GPIOC
#define USART_TX_Pin 	GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin 	GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin 		GPIO_PIN_5
#define LD2_GPIO_Port 	GPIOA
#define TMS_Pin 		GPIO_PIN_13
#define TMS_GPIO_Port 	GPIOA
#define TCK_Pin 		GPIO_PIN_14
#define TCK_GPIO_Port 	GPIOA
#define SWO_Pin 		GPIO_PIN_3
#define SWO_GPIO_Port 	GPIOB
/* USER CODE BEGIN Private defines */

#define FLASH_SECTOR2_BASE_ADDRESS (0x08008000U)
#define DBGMCU_IDCODE ((volatile uint32_t *)0xE0042000)
#define OPTION_BYTES_BASE_ADDRESS ((uint32_t *)(0x1FFFC000))

#define ADDRESS_VALID 0x0
#define ADDRESS_INVALID 0x1

#define SRAM1_SIZE 112
#define SRAM2_SIZE 16
#define BKPSRAM_SIZE 4
#define SRAM1_END (SRAM1_BASE + (SRAM1_SIZE * 1024))
#define SRAM2_END (SRAM2_BASE + (SRAM2_SIZE * 1024))
#define BKPSRAM_END (BKPSRAM_BASE + (BKPSRAM_SIZE * 1024))

#define INVALID_SECTOR 0x1
/*
#define
#define
#define
#define
*/

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
