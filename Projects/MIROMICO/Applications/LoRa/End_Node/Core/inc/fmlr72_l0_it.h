/*
 ******************************************************************************
 * @file    mlm32l0xx_it.h
 * @brief   manages interupt
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L0xx_IT_H__
#define __STM32L0xx_IT_H__

#ifdef __cplusplus
extern "C" {
#endif

void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void RTC_IRQHandler(void);
void EXTI0_1_IRQHandler(void);
void EXTI2_3_IRQHandler(void);
void EXTI4_15_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32L0xx_IT_H__ */
