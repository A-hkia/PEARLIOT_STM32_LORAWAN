/**
  ******************************************************************************
  * @file    hw.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    15-November-2016
  * @brief   This file contains definitions for:
  *          - LEDs
  */

/* Includes ------------------------------------------------------------------*/
#include "hw_led.h"
#include "FMLR72_L0.h"
#include <hw_gpio.h>
#include <stdlib.h>

extern GPIO_TypeDef* LED_PORT[LEDn];
extern const uint16_t LED_PIN[LEDn];
extern const GPIO_PinState LED_OFF_STATE[LEDn];

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void HW_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

  if (Led < LEDn) {
    HW_GPIO_Init(LED_PORT[Led], LED_PIN[Led], &GPIO_InitStruct);
  }
  HW_LED_Off(Led);
}

/**
  * @brief  Deinitialize LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void HW_LED_DeInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

  if (Led < LEDn) {
    HW_GPIO_Init(LED_PORT[Led], LED_PIN[Led], &GPIO_InitStruct);
  }
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void HW_LED_On(Led_TypeDef Led)
{
  if (Led < LEDn) {
    HW_GPIO_Write(LED_PORT[Led], LED_PIN[Led], !LED_OFF_STATE[Led]);
  }
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void HW_LED_Off(Led_TypeDef Led)
{
  if (Led < LEDn) {
    HW_GPIO_Write(LED_PORT[Led], LED_PIN[Led], LED_OFF_STATE[Led]);
  }
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void HW_LED_Toggle(Led_TypeDef Led)
{
  if (Led < LEDn) {
    HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
  }
}
