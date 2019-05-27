/******************************************************************************
 * @file    hw_msp.h
 * @author  MCD Application Team
 * @version V1.0.2
 * @date    15-November-2016
 * @brief   Header for driver hw msp module
 ******************************************************************************
 */
#ifndef __HW_MSP_H__
#define __HW_MSP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */



/** Get hw id */
uint8_t HW_GetId();

/** Check for production key in audio connector */
bool CheckProductionState();

#ifdef __cplusplus
}
#endif

#endif /* __HW_MSP_H__ */
