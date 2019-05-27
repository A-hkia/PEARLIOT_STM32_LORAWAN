/******************************************************************************
 * @file    board_conf.h
 * @brief   contains board specific configuration
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_CONF_H__
#define __BOARD_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/** Define battery limits for current hardware */
#define VDDA_VREFINT_CAL         	((uint32_t) 3000)
#define VDD_BAT                  	((uint32_t) 3000)
#define VDD_MIN                  	1800

// LED definitions
typedef enum {
  LED1 = 0,
  LED_RED = LED1,
  LED2 = 1,
  LED_GREEN = LED2,
  LED3 = 2,
  LED_BLUE = LED3,
} Led_TypeDef;

#define LEDn                              0

#define LED1_PIN                          GPIO_PIN_2
#define LED1_GPIO_PORT                    GPIOA
#define LED1_OFF                          GPIO_PIN_SET

#define LED2_PIN                          GPIO_PIN_5
#define LED2_GPIO_PORT                    GPIOA
#define LED2_OFF                          GPIO_PIN_SET

#define LED3_PIN                          GPIO_PIN_3
#define LED3_GPIO_PORT                    GPIOA
#define LED3_OFF                          GPIO_PIN_SET

#define US100_PWR_PIN                     GPIO_PIN_0
#define US100_PWR_PORT                    GPIOA

/** SPI2: External flash */
#define SPI2_NSS_PORT                 GPIOB
#define SPI2_NSS_PIN                  GPIO_PIN_12

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_CONF_H__ */
