/*
 ******************************************************************************
 * @file    mlm32l0xx_it.h
 * @brief   manages interupt
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "fmlr72_l0_it.h"
#include "hw_uart.h"
#include "hw_rtc.h"
#include "stm32l0xx_ll_usart.h"

//void Reset_Handler(void) {
//	while(1);
//}
/******************************************************************************/
/*                 STM32L1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx.s).                                               */
/******************************************************************************/

void USART1_IRQHandler(void)
{
  // Check RXNE flag value in ISR register
//  if (LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1)) {
    // RXNE flag will be cleared by reading of RDR register (done in call)
    // Call function in charge of handling Character reception
    //HW_UART_Rx_Callback(USART1);
	HAL_UART_IRQHandler(USART1);
 // } else {
    // Error_Callback();
  //}
}

void USART2_IRQHandler(void)
{
  // Check RXNE flag value in ISR register
  //if (LL_USART_IsActiveFlag_RXNE(USART2) && LL_USART_IsEnabledIT_RXNE(USART2)) {
    // RXNE flag will be cleared by reading of RDR register (done in call)
    // Call function in charge of handling Character reception
    //HW_UART_Rx_Callback(USART2);
	HAL_UART_IRQHandler(USART1);
  //} else {
    // Error_Callback();
  //}
}

void RTC_IRQHandler(void)
{
  HW_RTC_IrqHandler();
}

void EXTI0_1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

void EXTI2_3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

void EXTI4_15_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}
