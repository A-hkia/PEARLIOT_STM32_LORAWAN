 /******************************************************************************
  * @file    vcom.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    10-July-2018
  * @brief   manages virtual com port
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "hw.h"
#include "vcom.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Uart Handle */
extern USART_HandleTypeDef husart1;

uint8_t charRx;

static void (*TxCpltCallback) (void);

static void (*RxCpltCallback) (uint8_t *rxChar);
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/
void vcom_Init(  void (*TxCb)(void) )
{

  /*Record Tx complete for DMA*/
  TxCpltCallback=TxCb;
  /*## Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = ODD parity
      - BaudRate = 921600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  husart1.Instance = USART1;
  husart1.Init.BaudRate = 115200;
  husart1.Init.WordLength = USART_WORDLENGTH_8B;
  husart1.Init.StopBits = USART_STOPBITS_1;
  husart1.Init.Parity = USART_PARITY_NONE;
  husart1.Init.Mode = USART_MODE_TX_RX;
  husart1.Init.CLKPolarity = USART_POLARITY_LOW;
  husart1.Init.CLKPhase = USART_PHASE_1EDGE;
  husart1.Init.CLKLastBit = USART_LASTBIT_DISABLE;

  if(HAL_USART_Init(&husart1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

void vcom_Trace(  uint8_t *p_data, uint16_t size )
{
  HAL_USART_Transmit_DMA(&husart1,p_data, size);
}


void HAL_USART_TxCpltCallback(USART_HandleTypeDef *husart1)
{
  /* buffer transmission complete*/
   if (NULL != TxCpltCallback)
   {
     TxCpltCallback();
   }
}

//void vcom_ReceiveInit(  void (*RxCb)(uint8_t *rxChar) )
//{
//  UART_WakeUpTypeDef WakeUpSelection;
//
//  /*record call back*/
//  RxCpltCallback=RxCb;
//
//  /*Set wakeUp event on start bit*/
//  WakeUpSelection.WakeUpEvent=UART_WAKEUP_ON_STARTBIT;
//
//  HAL_UARTEx_StopModeWakeUpSourceConfig(&husart1, WakeUpSelection );
//
//  /*Enable wakeup from stop mode*/
//  HAL_UARTEx_EnableStopMode(&husart1);
//
//  /*Start LPUART receive on IT*/
//  HAL_USART_Receive_IT(&husart1, &charRx,1);
//}

void HAL_USART_RxCpltCallback(USART_HandleTypeDef *husart1)
{
   if ((NULL != RxCpltCallback) && (HAL_USART_ERROR_NONE ==husart1->ErrorCode))
   {
     RxCpltCallback(&charRx);
   }
   HAL_USART_Receive_IT(husart1, &charRx,1);
}


void vcom_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(husart1.hdmatx);
}

void vcom_IRQHandler(void)
{
  HAL_USART_IRQHandler(&husart1);
}

void vcom_DeInit(void)
{
  HAL_USART_DeInit(&husart1);
}


void vcom_IoInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct={0};
    /* Enable GPIO TX/RX clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();
    /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;

  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Alternate = USARTx_RX_AF;

  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
}

void vcom_IoDeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure={0};

  USARTx_TX_GPIO_CLK_ENABLE();

  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

  GPIO_InitStructure.Pin =  USARTx_TX_PIN ;
  HAL_GPIO_Init(  USARTx_TX_GPIO_PORT, &GPIO_InitStructure );

  GPIO_InitStructure.Pin =  USARTx_RX_PIN ;
  HAL_GPIO_Init(  USARTx_RX_GPIO_PORT, &GPIO_InitStructure );
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
