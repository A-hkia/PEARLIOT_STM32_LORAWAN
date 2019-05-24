/*
 ******************************************************************************
 * @file    hw_conf.h
 * @author  MCD Application Team
 * @version V1.1.1
 * @date    01-June-2017
 * @brief   contains hardware configuration Macros and Constants
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONF_H__
#define __HW_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#if defined(CMWX1ZZABZ)
#include "CMWX1ZZABZ.h"
#elif defined(FMLR72_L0)
#include "FMLR72_L0.h"
#else
#error Define module type
#endif
#include "board_conf.h"

#include "hw_gpio.h"
#include "hw_spi.h"
#include "hw_rtc.h"
#include "hw_uart.h"
#include "hw_led.h"
#include "hw_power.h"
#include "hw_i2c.h"
#include "hw_id.h"
#include "hw_adc.h"
#include "hw_eeprom.h"
#include "hw_msp.h"

#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_usart.h"

/* --------Preprocessor compile swicth------------ */
/* debug swicthes in debug.h */
//#define TRACE

/* uncomment below line to never enter lowpower modes in main.c*/
//#define LOW_POWER_DISABLE

#define USE_ADC
#define USE_RTC
#define USE_I2C1

#define USE_USART1
#define USE_USART2

// Usage of SPI is module and application depended.
// Make sure LoRa SPI is enabled
#if defined(CMWX1ZZABZ)
// LoRa
#define USE_SPI1
// not used
#undef USE_SPI2
#elif defined(FMLR72_L0)
// external flash
#define USE_SPI1
// LoRa
#define USE_SPI2
#else
#error Define module type
#endif

#ifdef __cplusplus
}
#endif

#endif /* __HW_CONF_H__ */
