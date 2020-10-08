/* USER CODE BEGIN Header */
//#define   _ECHOBACK
//#define	_DEBUGPRINT_SUM
//#define _DEBUFPRINT_UART1_RX
//#define	_DEBUGPRINT_COB_ENC
//#define	_DEBUGPRINT_COB_DEC
//#define	_DEBUGPRINT_RX_PARSE
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
#include "dma.h"
#include "tim.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

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
#define	LED_ON0()		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define	LED_ON1()		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
#define	LED_OFF0()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define	LED_OFF1()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)

#define	SW_Read()	    HAL_GPIO_ReadPin(PushSW_GPIO_Port, PushSW_Pin)

//### for cobs
#ifndef ArrayLength
#define ArrayLength(X)    (sizeof(X)/sizeof((X)[0]))
#endif

//### for USART1_EventHandler()
//uint8_t SCI_TX_DAT[TxBuff_Density] = {0x0};
uint8_t SCI_RX_DAT[RxBuff_Density] = {0x0};

uint8_t USART1_RX_BUF[RxBuff_Density] = {0x0};
#define DMA_WRITE_PTR ( (RxBuff_Density - huart1.hdmarx->Instance->CNDTR) % (RxBuff_Density) )

//### for LED_CTRL()
uint8_t LED[2];   

//### Reserved
uint8_t	SW0 = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//----------------------------------------------------------------
// UART Rx (w/DMA, for data) 921.6kbps
//----------------------------------------------------------------
void USART1_EventHandler(void)
{
    uint8_t rxlen = 0;
    uint8_t i;

    // idle状態検出
   	if((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) && (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_IDLE) != RESET))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UART_DMAStop(&huart1);      // DMA受信停止

        rxlen = DMA_WRITE_PTR;  // 受信データ長
#ifdef _DEBUFPRINT_UART1_RX
        xprintf("UART1_RxLength = %d\n", rxlen);
        xputs("DAT = ");
#endif
        for (i = 0; i < rxlen; i++)
        {
        	SCI_RX_DAT[i] = USART1_RX_BUF[i];
#ifdef _DEBUFPRINT_UART1_RX
        	xprintf("%02X ",SCI_RX_DAT[i]);
#endif

          if(SCI_RX_DAT[i] == 0x00)  //  データ末尾
          {
            UART_Rx_Decode(&SCI_RX_DAT, i);
          }
        }
#ifdef _DEBUFPRINT_UART1_RX
        xputs("\n");
#endif
        HAL_UART_Receive_DMA(&huart1, USART1_RX_BUF, RxBuff_Density);  // DMA受信再スタート
    }
}

//----------------------------------------------------------------
// Convert cobsr to plain
//----------------------------------------------------------------
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
    print_hex(in_buffer, Receive_Length -1);
    xprintf("\nLength %u, Status %02X\n", result_rx.out_len, result_rx.status);
#endif
    uint8_t sum = in_buffer[in_buffer[0] -1];
#ifdef  _DEBUGPRINT_COB_DEC
    xprintf("\nSUM from packet : %02X\n",sum);
#endif
//    if(in_buffer[Receive_Length - 1] != check_sum(&in_buffer, Receive_Length - 1))
    if(sum != check_sum(&in_buffer, in_buffer[0] -1) )
    {
    	xputs("###Rx_Decode : Sum is incorrect\n");
    	xputs("\n");
    	xprintf("SUM Calc : %02X, Received : %02X\n", check_sum(&in_buffer, in_buffer[0] -1), sum);

        xputs("Received data : \n");
        print_hex(out_buffer, Receive_Length + 2);
        xprintf("\nLength %u\n", Receive_Length + 2);

    	xputs("Decoded data : \n");
        print_hex(in_buffer, in_buffer[0]);
        xprintf("\nLength %u, Status %02X\n", result_rx.out_len + 1, result_rx.status);
      return;   // Checksums don't match, abort.
    }

#ifdef  _DEBUGPRINT_COB_DEC
    uint8_t data_length = in_buffer[0]-(TxHeader_Length+1);
    xprintf("Src_Addr : 0x%02X, Dest_Addr : 0x%02X, Command : 0x%02X, Data_Length : %d\n", in_buffer[1], in_buffer[2], in_buffer[3], data_length);
    xputs("Payload : \n");
    print_hex(&in_buffer[TxHeader_Length], data_length);
    xputs("\n");
#endif

    UART_Rx_Parse(&in_buffer, in_buffer[0]);
}

//----------------------------------------------------------------
// parsing Rx Data
//----------------------------------------------------------------
void UART_Rx_Parse(uint8_t *buf, uint16_t len)
{
  uint8_t SRC_ADDR = buf[1];
  uint8_t DEST_ADDR = buf[2];
  uint8_t COMMAND = buf[3];
  uint8_t i;

  uint8_t PAYLOAD[256];
  uint8_t PAYLOAD_LENGTH = buf[0]-(TxHeader_Length+1);

  memset(PAYLOAD, 0x00, sizeof(PAYLOAD));

	// パケ?��?トフィルタ
	if(OWN_ADDRESS == SRC_ADDR)	// Loopback detect
	{
#ifdef _DEBUGPRINT_RX_PARSE
	  xputs("own Packet!\n");
#endif
	  return;
	}

	if(OWN_ADDRESS != DEST_ADDR)	// Packets not related to the own station
	{
		if(DEST_ADDR == 0x0)	//Broadcast packets are receive
		{
#ifdef _DEBUGPRINT_RX_PARSE
			xputs("Broadcast Packet!\n");
#endif
		} else
		{
#ifdef _DEBUGPRINT_RX_PARSE
			xputs("wasted Packet!\n");
#endif
			  return;
		}
	}

	for(i = 0; i < (buf[0]-(TxHeader_Length+1)); i++)
	{
		PAYLOAD[i] = buf[TxHeader_Length + i];
	}

	LED_CTRL(LED_SETTIME,1,LED_PERSIST_TIME);	// Rx LED
#ifdef _DEBUGPRINT_RX_PARSE
	xprintf("COMMAND = %02X\n", COMMAND);
#endif
	switch(COMMAND)
	{
		case CMD_NOP:
#ifdef _DEBUGPRINT_RX_PARSE
			xputs("### CMD_NOP ###\n");
			xputs("Payload : \n");
			print_hex(&PAYLOAD, PAYLOAD_LENGTH);
			xputs("\n");
#endif
			break;

		default:
			xputs("###Parse : Wrong Command \n");
			xprintf("Src_Addr : 0x%02X, Dest_Addr : 0x%02X, Command : 0x%02X\n", SRC_ADDR, DEST_ADDR, COMMAND);
			xputs("Payload : \n");
			print_hex(&PAYLOAD, PAYLOAD_LENGTH);
			xputs("\n");
			break;
	}
}

//----------------------------------------------------------------
// UART Tx (w/DMA, for data) 921.6kbps
//----------------------------------------------------------------
void UART_Transmit(uint8_t chara)
{
//	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&chara, 1);
	LED_CTRL(LED_SETTIME,0,LED_PERSIST_TIME);	// Tx LED
	HAL_UART_Transmit(&huart1, (uint8_t*)&chara, 1, 100);
	return;
}

//----------------------------------------------------------------
// UART Tx (w/DMA, for Debug) 115.2kbps
//----------------------------------------------------------------
void UART_Transmit_DebugOut(uint8_t chara)
{
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

//---------------------------------------------------------------------------------------------------
// Dump any length
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
// send uart1 any length
//---------------------------------------------------------------------------------------------------
void UART_Tx_bin(const uint8_t * src_ptr, size_t src_len)
{
    size_t	i;

    for (i = 0; i < src_len; i++)
    {
    	UART_Transmit(*src_ptr++);
    }
}

//---------------------------------------------------------------------------------------------------
// Convert data to packets and send (UART1)
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

    // memset(SCI_TX_DAT, 0xFF, sizeof(SCI_TX_DAT));   // TxBuff Clear

    // for(i=0; i<result.out_len + 1; i++)
    // {
    //   SCI_TX_DAT[i] = out_buffer[i];
    // }

	UART_Tx_bin(out_buffer, result.out_len + 1);	// Send to UART
}


//---------------------------------------------------------------------------------------------------
// Set ON-Persist time to LED
//
//LED_CTRL(LED_DOWNCOUNT,0,0);
//LED_CTRL(LED_SETTIME,0,LED_PERSIST_TIME);
//---------------------------------------------------------------------------------------------------
void LED_CTRL( uint8_t control, uint8_t channel, uint8_t time)
{
    uint8_t i;

    switch(control)
    {
      case LED_DOWNCOUNT:
          for(i = 0; i < LED_MAX_CHANNEL; i++)
          {
//        	  xprintf("LED_Channel = %d\n",i);
              if(LED[i] == 0)   // off state
              {
                  if(i == 0)
                  {
                      LED_OFF0();
//                      break;
                  } else if(i == 1)
                  {
                      LED_OFF1();
//                      break;
                  }
              } else
              {
                LED[i] = LED[i] -1;   // on state (count down)

//                xprintf("LED%d = %d\n",i,LED[i]);

                if(i == 0)
                {
                    LED_ON0();
//                    break;
                } else if(i == 1)
                {
                    LED_ON1();
//                    break;
                }
              }
          }
          break;

      case LED_SETTIME:
          LED[channel] = time;    // set to LED-on time
          break;

      default:
          break;
    }
}

//----------------------------------------------------------------
// cyclic timer interrupt (10mSec)
//
//	htim3.Init.Prescaler = 3200-1;
//	htim3.Init.Period = 200;
//	APB2 = 64MHz
//	1 / (64M / (3200 x 200)) = 10e-3[Sec]
//
//----------------------------------------------------------------
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{

  if((htim->Instance == TIM3) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
  {
    // Timeout_Function
    SW0 = SW_Read();

    LED_CTRL(LED_DOWNCOUNT,0,0);

  }
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim3);
	__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);

	__HAL_UART_DISABLE_IT(&huart1, UART_IT_PE);
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_ERR);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, USART1_RX_BUF, RxBuff_Density);

	xfunc_out = UART_Transmit_DebugOut;	//uart2, 115.2kbps

	uint8_t UART_Tx_Buf[32] = {0x55, 0xAA, 0xFF, 0x12, 0x34, 0xDE, 0xAD, 0xBE, 0xEF};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

	UART_Tx_Buf[0] = rand() % 0xFF + 1;
	UART_Tx_Buf[1] = rand() % 0xFF + 1;
	UART_Tx_Buf[2] = rand() % 0xFF + 1;
	UART_Tx_Buf[3] = rand() % 0xFF + 1;
	UART_Tx_Buf[4] = rand() % 0xFF + 1;
	UART_Tx_Buf[5] = rand() % 0xFF + 1;
	UART_Tx_Buf[6] = rand() % 0xFF + 1;
	UART_Tx_Buf[7] = rand() % 0xFF + 1;

    HAL_Delay(500);	// 自分自身からのパケット
	UART_Tx_w_encode(UART_Tx_Buf, 9, OWN_ADDRESS, 0x10, CMD_NOP);  // buf, buf_length(count from 1), Src_Addr, Dest_Addr, Command
//	LED_CTRL(LED_SETTIME,0,LED_PERSIST_TIME);	// Tx LED

	UART_Tx_Buf[0] = rand() % 0xFF + 1;
	UART_Tx_Buf[1] = rand() % 0xFF + 1;
	UART_Tx_Buf[2] = rand() % 0xFF + 1;
	UART_Tx_Buf[3] = rand() % 0xFF + 1;
	UART_Tx_Buf[4] = rand() % 0xFF + 1;
	UART_Tx_Buf[5] = rand() % 0xFF + 1;
	UART_Tx_Buf[6] = rand() % 0xFF + 1;
	UART_Tx_Buf[7] = rand() % 0xFF + 1;
	HAL_Delay(500);	// 他局からのパケット(他局宛)
	UART_Tx_w_encode(UART_Tx_Buf, 9, 0x02, 0x10, CMD_NOP);  // buf, buf_length(count from 1), Src_Addr, Dest_Addr, Command

	UART_Tx_Buf[0] = rand() % 0xFF + 1;
	UART_Tx_Buf[1] = rand() % 0xFF + 1;
	UART_Tx_Buf[2] = rand() % 0xFF + 1;
	UART_Tx_Buf[3] = rand() % 0xFF + 1;
	UART_Tx_Buf[4] = rand() % 0xFF + 1;
	UART_Tx_Buf[5] = rand() % 0xFF + 1;
	UART_Tx_Buf[6] = rand() % 0xFF + 1;
	UART_Tx_Buf[7] = rand() % 0xFF + 1;
	HAL_Delay(500);	// 他局からのパケット(自局宛)
	UART_Tx_w_encode(UART_Tx_Buf, 9, 0x02, OWN_ADDRESS, CMD_NOP);  // buf, buf_length(count from 1), Src_Addr, Dest_Addr, Command
//	LED_CTRL(LED_SETTIME,1,LED_PERSIST_TIME);	// Rx LED

	UART_Tx_Buf[0] = rand() % 0xFF + 1;
	UART_Tx_Buf[1] = rand() % 0xFF + 1;
	UART_Tx_Buf[2] = rand() % 0xFF + 1;
	UART_Tx_Buf[3] = rand() % 0xFF + 1;
	UART_Tx_Buf[4] = rand() % 0xFF + 1;
	UART_Tx_Buf[5] = rand() % 0xFF + 1;
	UART_Tx_Buf[6] = rand() % 0xFF + 1;
	UART_Tx_Buf[7] = rand() % 0xFF + 1;
	HAL_Delay(500);	// グローバルパケット
	UART_Tx_w_encode(UART_Tx_Buf, 9, 0x02, 0x00, CMD_NOP);  // buf, buf_length(count from 1), Src_Addr, Dest_Addr, Command

//		LED_ON();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
