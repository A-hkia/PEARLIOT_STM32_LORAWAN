/**
  ******************************************************************************
  * @file    hw_led.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    15-November-2016
  * @brief   This file contains definitions for:
  *          - LEDs
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_LED_H
#define __HW_LED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "stdlib.h"

void HW_LED_Init(Led_TypeDef Led);
void HW_LED_DeInit(Led_TypeDef Led);
void HW_LED_On(Led_TypeDef Led);
void HW_LED_Off(Led_TypeDef Led);
void HW_LED_Toggle(Led_TypeDef Led);

#ifdef __cplusplus
}
#endif

#endif
