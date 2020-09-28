/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void USART1_EventHandler(void);
void UART_Rx_Decode(uint8_t *buf, uint16_t len);
void UART_Rx_Parse(uint8_t *buf, uint16_t len);
void UART_Transmit(uint8_t chara);
void UART_Transmit_DebugOut(uint8_t chara);
uint8_t check_sum(uint8_t *buf, uint16_t len);
void print_hex(const uint8_t * src_ptr, size_t src_len);
void UART_Tx_bin(const uint8_t * src_ptr, size_t src_len);
void UART_Tx_w_encode(const uint8_t * src_ptr, size_t src_len, uint8_t SRC_ADDR, uint8_t DEST_ADDR, uint8_t COMMAND);

void LED_CTRL( uint8_t control, uint8_t channel, uint8_t time);

HAL_StatusTypeDef HAL_UART_DMA_Tx_Stop(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define PushSW_Pin GPIO_PIN_5
#define PushSW_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define TxBuff_Density 257
#define RxBuff_Density 257
//#define str2dec_buf_size 1020

#define LED_MAX_CHANNEL 2
#define LED_PERSIST_TIME  5
#define LED_DOWNCOUNT 0
#define LED_SETTIME 1

#define OWN_ADDRESS 1
#define TxHeader_Length 4
#define CMD_NOP 0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
