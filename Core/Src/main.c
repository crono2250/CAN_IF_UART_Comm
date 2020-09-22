/* USER CODE BEGIN Header */
//#define   _ECHOBACK
#define	_DEBUGPRINT_SUM
#define	_DEBUGPRINT_COB_ENC
#define	_DEBUGPRINT_COB_DEC

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdlib.h> 		// for atol()
#include <stdio.h>
#include <string.h>			// for memset()
#include "cobsr.h"
#include "xprintf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	LED_ON()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define	LED_OFF()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)

#define	SW_Read()	HAL_GPIO_ReadPin(PushSW_GPIO_Port, PushSW_Pin)

#ifndef ArrayLength			// for cobs
#define ArrayLength(X)    (sizeof(X)/sizeof((X)[0]))
#endif

uint8_t	SW0 = 0;

uint8_t	RxData = 0;

uint16_t SCI_BYTES_READ = 0x0;
uint8_t SCI_TX_DAT[TxBuff_Density] = {0x0};
uint8_t SCI_RX_DAT[RxBuff_Density] = {0x0};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//----------------------------------------------------------------
// UART Rx (w/Interrupt)
//----------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	uint8_t d = 0x0;
  uint8_t Byte_Count = 0;

  if(UartHandle->Instance == USART1)
  {
	 	d = RxData;

	 	SCI_RX_DAT[SCI_BYTES_READ] = d;
	 	SCI_BYTES_READ++;
	 #ifdef  _ECHOBACK
	 		  xprintf("%c", d);  // echo back
	 #endif

	   if(d == 0x00)  //  データ末尾
	   {
	 	Byte_Count = SCI_BYTES_READ;  //  バッファにコピーして
	 	SCI_BYTES_READ = 0;           //  受信カウントクリア
	 	UART_Rx_Decode(&SCI_RX_DAT, Byte_Count);
	   }

	   HAL_UART_Receive_IT(&huart1, &RxData, 1);
}

  if(UartHandle->Instance == USART2)
  {
	  //Not use
  }

  return;
}

void UART_Rx_Decode(uint8_t *buf, uint16_t len)
{
    cobsr_decode_result   result_rx;
    uint8_t               in_buffer[257];
    uint8_t               out_buffer[257];
    uint8_t               i;
    uint8_t               Receive_Length = 0;

    memset(in_buffer, 'A', sizeof(in_buffer));
    memset(out_buffer, 'A', sizeof(in_buffer));

    Receive_Length = len;     

    for(i = 0; i < Receive_Length; i++)
    {
    	out_buffer[i] = buf[i];
    }

#ifdef  _DEBUGPRINT_COB_DEC
    xprintf("Length = %d\n", Receive_Length);
#endif
    result_rx = cobsr_decode(in_buffer, sizeof(in_buffer), out_buffer, Receive_Length);
#ifdef  _DEBUGPRINT_COB_DEC
    xputs("Decoded data : \n");
    print_hex(in_buffer, Receive_Length);
    xprintf("\nLength %u, Status %02X\n", result_rx.out_len, result_rx.status);
#endif
    uint8_t sum = in_buffer[in_buffer[0] -1];
    xprintf("\nSUM from packet : %02X\n",sum);
//    if(in_buffer[Receive_Length - 1] != check_sum(&in_buffer, Receive_Length - 1))
    if(sum != check_sum(&in_buffer, in_buffer[0]) )
    {
    	xputs("Sum is incorrect\n");
    	xputs("\n");
    	xprintf("SUM Calc : %02X, Received : %02X\n", check_sum(&in_buffer, in_buffer[0]), sum);

        xputs("Received data : \n");
        print_hex(out_buffer, Receive_Length + 2);
        xprintf("\nLength %u\n", Receive_Length + 2);

    	xputs("Decoded data : \n");
        print_hex(in_buffer, in_buffer[0]);
        xprintf("\nLength %u, Status %02X\n", result_rx.out_len + 1, result_rx.status);
      return;   // sum合わず失敗した
    }

    uint8_t data_length = in_buffer[0]-(TxHeader_Length+1);
    xprintf("Src_Addr : 0x%02X, Dest_Addr : 0x%02X, Command : 0x%02X, Data_Length : %d\n", in_buffer[1], in_buffer[2], in_buffer[3], data_length);
    xputs("Payload : \n");
    print_hex(&in_buffer[4], data_length);
    xputs("\n");

    UART_Rx_Parse(&in_buffer, in_buffer[0]);
}

void UART_Rx_Parse(uint8_t *buf, uint16_t len)
{
  uint8_t SRC_ADDR = buf[1];
  uint8_t DEST_ADDR = buf[2];
  uint8_t COMMAND = buf[3];
  uint8_t i;

  uint8_t PAYLOAD[256];
  uint8_t PAYLOAD_LENGTH = buf[0]-(TxHeader_Length+1);

  // パケ?��?トフィルタ
  if(OWN_ADDRESS == SRC_ADDR)   return;   // 自分自身が送ったパケットだった
  if(OWN_ADDRESS != DEST_ADDR || DEST_ADDR != 0x0)  return;   // 自分に関係ないパケットだった。 ただし同報パケットは通す

    for(i = 0; i < (buf[0]-(TxHeader_Length+1)); i++)
    {
    	PAYLOAD[i] = buf[4+i];
    }

  switch(COMMAND)
  {
    case CMD_NOP:
        xputs("Payload : \n");
        print_hex(&PAYLOAD, PAYLOAD_LENGTH);
        xputs("\n");
        break;
    
    default:
        break;
  }
}

//----------------------------------------------------------------
// UART Tx (w/DMA, for data) 921.6kbps
//----------------------------------------------------------------
void UART_Transmit(uint8_t chara)
{
//	HAL_UART_Transmit_IT(&huart1, &chara, 1);
//	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&chara, 1);
	HAL_UART_Transmit(&huart1, (uint8_t*)&chara, 1, 100);
	return;
}

//----------------------------------------------------------------
// UART Tx (w/DMA, for Debug) 115.2kbps
//----------------------------------------------------------------
void UART_Transmit_DebugOut(uint8_t chara)
{
//	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&chara, 1);
	HAL_UART_Transmit(&huart2, (uint8_t*)&chara, 1, 100);
	return;
}

//----------------------------------------------------------------
// checksum
//----------------------------------------------------------------
uint8_t check_sum(uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint32_t sum = 0;

#ifdef  _DEBUGPRINT_SUM
    xputs("\nSUM : ");
#endif

    for(i=0;i<len;i++)
    {
#ifdef  _DEBUGPRINT_SUM
    	xprintf("%02X ", buf[i]);
#endif
    	sum+= buf[i];
    }

    sum = 0x000000FF & sum;

#ifdef  _DEBUGPRINT_SUM
    xprintf("\nsum : %02X\n", (uint8_t)sum);
#endif

    return (uint8_t)sum;
}

//----------------------------------------------------------------
// cyclic timer interrupt
//----------------------------------------------------------------
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{

  if((htim->Instance == TIM3) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
  {
    // Timeout_Function
    SW0 = SW_Read();

  }
}

//---------------------------------------------------------------------------------------------------
// 任意長ダンプ
//---------------------------------------------------------------------------------------------------
void print_hex(const uint8_t * src_ptr, size_t src_len)
{
    size_t              i;


    for (i = 0; i < src_len; i++)
    {
        xprintf("%02X ", *src_ptr++);
    }
}

//---------------------------------------------------------------------------------------------------
// 任意長でバッファ内のデータをそのまま送信
//---------------------------------------------------------------------------------------------------
void UART_Tx_bin(const uint8_t * src_ptr, size_t src_len)
{
    size_t              i;

    for (i = 0; i < src_len; i++)
    {
    	UART_Transmit(*src_ptr++);
    }
}

//---------------------------------------------------------------------------------------------------
// データをパケットに加工・送信
//---------------------------------------------------------------------------------------------------
void UART_Tx_w_encode(const uint8_t * src_ptr, size_t src_len, uint8_t SRC_ADDR, uint8_t DEST_ADDR, uint8_t COMMAND)
{
    uint8_t             out_buffer[257];
    cobsr_encode_result  result;
    uint8_t   TxData[32];
    uint8_t   i;

    memset(TxData, 0x00, sizeof(TxData));
    memset(out_buffer, 'A', sizeof(out_buffer));

    for (i = 0; i < src_len; i++)
    {
      TxData[TxHeader_Length + i]  = *src_ptr++;	// 4 = Length(1) + Src_Addr(1) + Dest_Addr(1) + Command(1)
    }

    // TxData[0] = 14;       //LENGTH
    TxData[0] = TxHeader_Length + 1 + i;       //LENGTH, 5 = Length(1) + Src_Addr(1) + Dest_Addr(1) + Command(1) + Sum(1)
    TxData[1] = SRC_ADDR;		// Source_Address
    TxData[2] = DEST_ADDR;		// Dest_Address
    TxData[3] = COMMAND;		//Command
    TxData[TxData[0] - 1] = check_sum(&TxData, TxData[0] - 1);

#ifdef  _DEBUGPRINT_COB_ENC
    xputs("Encode data:\n");
    print_hex(TxData, TxData[0]);
    xputs("\n");
#endif
    result = cobsr_encode(out_buffer, sizeof(out_buffer), TxData, TxData[0]);
#ifdef  _DEBUGPRINT_COB_ENC
    xputs("Encoded data:\n");
    print_hex(out_buffer, result.out_len + 1);
    xprintf("\nLength %u, Status %02X\n", result.out_len + 1, result.status);
#endif

    memset(SCI_TX_DAT, 0xFF, sizeof(SCI_TX_DAT));   // TxBuff Clear

    for(i=0; i<result.out_len + 1; i++)
    {
      SCI_TX_DAT[i] = out_buffer[i];
    }

	UART_Tx_bin(out_buffer, result.out_len + 1);	// Send to UART

//-------
//### Receive


//     uint8_t	Receive_Length = 0;
//     for(i = 0; out_buffer[i] != 0; i++)
//     {
//     	Receive_Length++;
//     }
//     Receive_Length = Receive_Length - 1;
// #ifdef  _DEBUGPRINT_COB_DEC
//     xprintf("Length = %d\n", Receive_Length);
// #endif
//     result_rx = cobsr_decode(in_buffer, sizeof(in_buffer), out_buffer, Receive_Length);
// #ifdef  _DEBUGPRINT_COB_DEC
//     xputs("Decoded data : \n");
//     print_hex(in_buffer, Receive_Length);
//     xprintf("\nLength %u, Status %02X\n", result_rx.out_len, result_rx.status);
// #endif
//     if(in_buffer[Receive_Length - 1] != check_sum(&in_buffer, Receive_Length - 1))
//     {
//     	xputs("Sum is incorrect\n");
//     	xputs("\n");
//     	xprintf("SUM Calc : %02X, Received : %02X\n", check_sum(&in_buffer, Receive_Length - 1), in_buffer[Receive_Length - 1]);

//         xputs("Received data : \n");
//         print_hex(out_buffer, Receive_Length + 2);
//         xprintf("\nLength %u\n", Receive_Length + 2);

//     	xputs("Decoded data : \n");
//         print_hex(in_buffer, Receive_Length);
//         xprintf("\nLength %u, Status %02X\n", result_rx.out_len + 1, result_rx.status);

//     }

//         xprintf("Src_Addr : 0x%02X, Dest_Addr : 0x%02X, Command : 0x%02X, Data_Length : %d\n", in_buffer[1], in_buffer[2], in_buffer[3], in_buffer[0]-(TxHeader_Length+1));
//         xputs("Payload : \n");
//         print_hex(&in_buffer[4], in_buffer[0]-(TxHeader_Length+1));
//         xputs("\n");

}

//---------------------------------------------------------------------------------------------------

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
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);

  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
  HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);

  HAL_UART_Receive_IT(&huart1, &RxData, 1);

  xfunc_out = UART_Transmit_DebugOut;	//uart2, 115.2kbps

  //xputs("test test!! unko unko!!!\n");
  //  HAL_Delay(1);
  
  uint8_t UART_Tx_Buf[32] = {0x55, 0xAA, 0xFF, 0x12, 0x34, 0xDE, 0xAD, 0xBE, 0xEF};

  UART_Tx_w_encode(UART_Tx_Buf, 9, 0x01, 0x10, CMD_NOP);  // buf, buf_length(count from 1), Src_Addr, Dest_Addr, Command

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  LED_ON();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 32000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 225-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 225-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PushSW_Pin */
  GPIO_InitStruct.Pin = PushSW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PushSW_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
