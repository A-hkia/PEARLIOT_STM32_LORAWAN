/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* LORA I/O definition */
#define GET_LORA_SPI						HW_SPI2_GetHandle

#define RADIO_RESET_PORT                          GPIOC
#define RADIO_RESET_PIN                           GPIO_PIN_2

/* Radio mosi */
#define RADIO_MOSI_PORT                           GPIOB
#define RADIO_MOSI_PIN                            GPIO_PIN_15
#define RADIO_MOSI_AF                             GPIO_AF0_SPI2

/* Radio miso */
#define RADIO_MISO_PORT                           GPIOB
#define RADIO_MISO_PIN                            GPIO_PIN_14
#define RADIO_MISO_AF                             GPIO_AF0_SPI2

/* Radio sclk */
#define RADIO_SCLK_PORT                           GPIOB
#define RADIO_SCLK_PIN                            GPIO_PIN_10
#define RADIO_SCLK_AF                             GPIO_AF5_SPI2

#define SPI_CLK_ENABLE()             			 __HAL_RCC_SPI2_CLK_ENABLE()

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
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
