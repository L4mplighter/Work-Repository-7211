/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
uint8_t ch_buf_226[226] = {0x00};
uint8_t zero = 0;

extern SCH1_result sch_data;
extern MMC5983_Data_TypeDef mmc_data;
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Stream0;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void SendData_226Byte(void)
{
	static int num_send = 0;
	num_send++;

	ch_buf_226[0] = 0xEB;
	ch_buf_226[1] = 0x90;

	ch_buf_226[2] = 0x08;
	ch_buf_226[3] = 0x65;
	                     
	memcpy(&ch_buf_226[4] , &sch_data.Acc2[0] , 4);	
	memcpy(&ch_buf_226[8] , &sch_data.Acc2[1] , 4);   
	memcpy(&ch_buf_226[12], &sch_data.Acc2[2] , 4);   
	memcpy(&ch_buf_226[16], &sch_data.Rate2[0] , 4);	
	memcpy(&ch_buf_226[20], &sch_data.Rate2[1] , 4);
	memcpy(&ch_buf_226[24], &sch_data.Rate2[2] , 4);
	
	memcpy(&ch_buf_226[28], &mmc_data.axes.DX , 4);			
	memcpy(&ch_buf_226[32], &mmc_data.axes.DY , 4); 
	memcpy(&ch_buf_226[36], &mmc_data.axes.DZ , 4); 
	memcpy(&ch_buf_226[40], &zero, 4);				
	memcpy(&ch_buf_226[44], &zero, 4);
	memcpy(&ch_buf_226[48], &zero, 4);
											 
	memcpy(&ch_buf_226[52], &zero, 4);
	memcpy(&ch_buf_226[56], &zero, 4);
	memcpy(&ch_buf_226[60], &zero, 4);
	memcpy(&ch_buf_226[64], &zero, 4);
	memcpy(&ch_buf_226[68], &zero, 4);
	memcpy(&ch_buf_226[72], &zero, 4);
	
	memcpy(&ch_buf_226[76], &zero, 4);
	memcpy(&ch_buf_226[80], &zero, 4);
	memcpy(&ch_buf_226[84], &zero, 4);
	memcpy(&ch_buf_226[88], &zero, 4);
	memcpy(&ch_buf_226[92], &zero, 4);
	memcpy(&ch_buf_226[96], &zero, 4);

	memcpy(&ch_buf_226[100], &zero, 4);			
	memcpy(&ch_buf_226[104], &zero, 4);
	memcpy(&ch_buf_226[108], &zero, 4);
	memcpy(&ch_buf_226[112], &zero, 4);			
	memcpy(&ch_buf_226[116], &zero, 4);          
	memcpy(&ch_buf_226[120], &zero, 4);          
					 
	memcpy(&ch_buf_226[124], &zero, 4);
	memcpy(&ch_buf_226[128], &zero, 4);
	memcpy(&ch_buf_226[132], &zero, 4);
	memcpy(&ch_buf_226[136], &zero, 4);
	memcpy(&ch_buf_226[140], &zero, 4); 
	memcpy(&ch_buf_226[144], &zero, 4); 
											 
	memcpy(&ch_buf_226[148], &zero, 4);
	memcpy(&ch_buf_226[152], &zero, 4);
	memcpy(&ch_buf_226[156], &zero, 4);
	memcpy(&ch_buf_226[160], &zero, 4);
	memcpy(&ch_buf_226[164], &zero, 4);
	memcpy(&ch_buf_226[168], &zero, 4);
	
	memcpy(&ch_buf_226[172], &zero, 4);					
	memcpy(&ch_buf_226[176], &zero, 4);
	memcpy(&ch_buf_226[180], &zero, 4);
	memcpy(&ch_buf_226[184], &zero, 4);						
	memcpy(&ch_buf_226[188], &zero, 4);
	memcpy(&ch_buf_226[192], &zero, 4);
	                                
	memcpy(&ch_buf_226[196], &zero, 4);					
	memcpy(&ch_buf_226[200], &zero, 4);
	memcpy(&ch_buf_226[204], &zero, 4);
	memcpy(&ch_buf_226[208], &zero, 4);
	memcpy(&ch_buf_226[212], &zero, 4);
	memcpy(&ch_buf_226[216], &zero, 4);
	
	memcpy(&ch_buf_226[220], &zero, 4);

	uint16_t check_sum;
	check_sum = 0;
	for (int i = 2; i < 224; i++)
	{
		check_sum = check_sum + ch_buf_226[i];
	}
	ch_buf_226[224] = check_sum & 0x00FF;
	ch_buf_226[225] = (check_sum >> 8) & 0x00FF;
	
	HAL_UART_Transmit_DMA(&huart1, ch_buf_226, 226);  
}

/* USER CODE END 1 */
