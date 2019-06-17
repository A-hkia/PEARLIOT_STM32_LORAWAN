/**
  ******************************************************************************
  * @file    CMWX1ZZABZ.h
  * @author  Alex Raimondi
  * @brief   This file contains definitions for CMWX1ZZABZ (Murata module)
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CMWX1ZZABZ_H
#define __CMWX1ZZABZ_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "hw_led.h"

/* LORA I/O definition */
#define GET_LORA_SPI						HW_SPI2_GetHandle

#define RADIO_RESET_PORT                          GPIOC
#define RADIO_RESET_PIN                           GPIO_PIN_2

/* Radio mosi */
#define SPI2_MOSI_PORT                           GPIOB
#define SPI2_MOSI_PIN                            GPIO_PIN_15
#define SPI2_MOSI_AF                             GPIO_AF0_SPI2

/* Radio miso */
#define SPI2_MISO_PORT                           GPIOB
#define SPI2_MISO_PIN                            GPIO_PIN_14
#define SPI2_MISO_AF                             GPIO_AF0_SPI2

/* Radio sclk */
#define SPI2_SCLK_PORT                           GPIOB
#define SPI2_SCLK_PIN                            GPIO_PIN_10
#define SPI2_SCLK_AF                             GPIO_AF5_SPI2

#define SPI2_CLK_ENABLE()             			 __HAL_RCC_SPI2_CLK_ENABLE()

#define RADIO_NSS_PORT                            GPIOA
#define RADIO_NSS_PIN                             GPIO_PIN_4

#define RADIO_DIO_0_PORT                          GPIOA
#define RADIO_DIO_0_PIN                           GPIO_PIN_1

#define RADIO_DIO_1_PORT                          GPIOA
#define RADIO_DIO_1_PIN                           GPIO_PIN_6

#define RADIO_DIO_2_PORT                          GPIOA
#define RADIO_DIO_2_PIN                           GPIO_PIN_7

#define RADIO_DIO_3_PORT                          GPIOC
#define RADIO_DIO_3_PIN                           GPIO_PIN_4

#ifdef RADIO_DIO_4
#define RADIO_DIO_4_PORT                          GPIOC
#define RADIO_DIO_4_PIN                           GPIO_PIN_5
#endif

#ifdef RADIO_DIO_5
#define RADIO_DIO_5_PORT                          GPIOB
#define RADIO_DIO_5_PIN                           GPIO_PIN_11
#endif

#define RADIO_ANTSW_PWR_PORT                      GPIOB
#define RADIO_ANTSW_PWR_PIN                       GPIO_PIN_12

/** SPI1: External flash */

#define SPI1_SCLK_PORT                GPIOB
#define SPI1_SCLK_PIN                 GPIO_PIN_3
#define SPI1_SCLK_AF                  GPIO_AF0_SPI1

#define SPI1_MISO_PORT                GPIOB
#define SPI1_MISO_PIN                 GPIO_PIN_4
#define SPI1_MISO_AF                  GPIO_AF0_SPI1

#define SPI1_MOSI_PORT                GPIOB
#define SPI1_MOSI_PIN                 GPIO_PIN_5
#define SPI1_MOSI_AF                  GPIO_AF0_SPI1

#define SPI1_CLK_ENABLE()             __HAL_RCC_SPI1_CLK_ENABLE()

#define SF_NSS_PORT                   GPIOB
#define SF_NSS_PIN                    GPIO_PIN_13

/* ---- I2C definitions ---- */

#define I2C1_SCL_PORT                GPIOB
#define I2C1_SCL_PIN                 GPIO_PIN_6
#define I2C1_SCL_AF                  GPIO_AF1_I2C1

#define I2C1_SDA_PORT                GPIOB
#define I2C1_SDA_PIN                 GPIO_PIN_9
#define I2C1_SDA_AF                  GPIO_AF4_I2C1

// On board LED definition
#define MLED_PIN                          GPIO_PIN_8
#define MLED_GPIO_PORT                    GPIOB
#define MLED_OFF                          GPIO_PIN_SET

/* --------------------------- USART HW definition -------------------------------*/

/* Definition for USART1 Pins */
#define USARTX                           USART1
#define USARTX_CLK_ENABLE()              __USART1_CLK_ENABLE();
#define USART1_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USART1_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USART1_TX_PIN                   GPIO_PIN_9
#define USART1_TX_GPIO_PORT             GPIOA
#define USART1_TX_AF                    GPIO_AF4_USART1
#define USART1_RX_PIN                   GPIO_PIN_10
#define USART1_RX_GPIO_PORT             GPIOA
#define USART1_RX_AF                    GPIO_AF4_USART1

/* Definition for USARTx's NVIC */
#define USART1_IRQn                     USART1_IRQn
#define USART1_IRQHandler               USART1_IRQHandler

/* Definition for USART2 Pins */
#define USART2_RX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART2_TX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART2_TX_PIN                  GPIO_PIN_3
#define USART2_TX_GPIO_PORT            GPIOA
#define USART2_TX_AF                   GPIO_AF4_USART2
#define USART2_RX_PIN                  GPIO_PIN_2
#define USART2_RX_GPIO_PORT            GPIOA
#define USART2_RX_AF                   GPIO_AF4_USART2


/* --------------------------- UART HW definition FROM BL072 - Needed for vcom.c-------------------------------*/

/* Definition for UARTx clock resources */
//#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __USART1_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()

#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF4_USART1
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF4_USART1

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler
/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA1_Channel7

/* Definition for USARTx's DMA Request */
#define USARTx_TX_DMA_REQUEST             DMA_REQUEST_5

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA1_Channel4_5_6_7_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA1_Channel4_5_6_7_IRQHandler

#define USARTx_Priority 1
#define USARTx_DMA_Priority 1


/* --------------------------- RTC HW definition -------------------------------- */

#define RTC_OUTPUT                      DBG_RTC_OUTPUT
#define RTC_Alarm_IRQn                  RTC_IRQn

#define ADCCLK_ENABLE()                 __HAL_RCC_ADC1_CLK_ENABLE();
#define ADCCLK_DISABLE()                __HAL_RCC_ADC1_CLK_DISABLE();

// LED convenience macros
#define LED_Toggle( x )                 HW_LED_Toggle( x );
#define LED_On( x )                     HW_LED_On( x );
#define LED_Off( x )                    HW_LED_Off( x );

uint32_t BSP_GetVersion(void);

#ifdef __cplusplus
}
#endif

#endif /* __CMWX1ZZABZ_H */
