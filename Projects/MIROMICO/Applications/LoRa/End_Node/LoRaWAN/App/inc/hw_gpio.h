/******************************************************************************
 * @file    hw_gpio.h
 * @author  MCD Application Team
 * @version V1.0.2
 * @date    15-November-2016
 * @brief   Header for driver hw_rtc.c module
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_GPIO_H__
#define __HW_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

typedef void(GpioIrqHandler)(void);

/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/*!
 * \brief GPIOs Macro
 */
#define RCC_GPIO_CLK_ENABLE( __GPIO_PORT__ )              \
do {                                                    \
    switch( __GPIO_PORT__)                                \
    {                                                     \
      case GPIOA_BASE: __HAL_RCC_GPIOA_CLK_ENABLE(); break;    \
      case GPIOB_BASE: __HAL_RCC_GPIOB_CLK_ENABLE(); break;    \
      case GPIOC_BASE: __HAL_RCC_GPIOC_CLK_ENABLE(); break;    \
      case GPIOD_BASE: __HAL_RCC_GPIOD_CLK_ENABLE(); break;    \
      case GPIOH_BASE: default:  __HAL_RCC_GPIOH_CLK_ENABLE(); \
    }                                                    \
  } while(0)

#define RCC_GPIO_CLK_DISABLE( __GPIO_PORT__ )              \
do {                                                    \
    switch( __GPIO_PORT__)                                \
    {                                                     \
      case GPIOA_BASE: __HAL_RCC_GPIOA_CLK_DISABLE(); break;    \
      case GPIOB_BASE: __HAL_RCC_GPIOB_CLK_DISABLE(); break;    \
      case GPIOC_BASE: __HAL_RCC_GPIOC_CLK_DISABLE(); break;    \
      case GPIOD_BASE: __HAL_RCC_GPIOD_CLK_DISABLE(); break;    \
      case GPIOH_BASE: default:  __HAL_RCC_GPIOH_CLK_ENABLE(); \
    }                                                    \
  } while(0)

#define NUM_PORTS 8

extern GPIO_TypeDef * const port_list[NUM_PORTS];

/**
 * Type to identify a specific gpio
 *
 * It is a integer number that allows calculation of the GPIO.
 * Divided by 16 => port with 0: GPIOA, 1 GPIOB, ...
 * Modulo 16 => pin in port.
 */
typedef uint16_t gpio_t;

/** Convert gpio_t to actual port */
#define PORT(x) port_list[(x) / 16]
/** convert gpio_t to actual pin */
#define PIN(x)  (1 << ((x) % 16))

#define TO_PORT_PIN(gpio) PORT(gpio), PIN(gpio)

#define PORT_PIN(port, pin) ((uint16_t)(port - 'A') * 16 + ((pin) & 0xFF))

/* Exported functions ------------------------------------------------------- */

/*!
 * GPIO IRQ handler function prototype
 */

IRQn_Type MSP_GetIRQn(uint16_t gpioPin);

/*!
 * @brief Initializes the given GPIO object
 *
 * @param  GPIOx: where x can be (A..E and H)
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @param [IN] initStruct  GPIO_InitTypeDef intit structure
 * @retval none
 */
static inline void HW_GPIO_Init(GPIO_TypeDef* port, uint16_t pin, GPIO_InitTypeDef* initStruct)
{
  RCC_GPIO_CLK_ENABLE((uint32_t)port);
  initStruct->Pin = pin;
  HAL_GPIO_Init(port, initStruct);
}

/*!
 * @brief Records the interrupt handler for the GPIO  object
 *
 * @param  GPIOx: where x can be (A..E and H)
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @param [IN] prio       NVIC priority (0 is highest)
 * @param [IN] irqHandler  points to the  function to execute
 * @retval none
 */
void HW_GPIO_SetIrq(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t prio,  GpioIrqHandler* irqHandler);

/*!
 * @brief Execute the interrupt from the object
 *
 * @param  GPIOx: where x can be (A..E and H)
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @retval none
 */
void HW_GPIO_IrqHandler(uint16_t GPIO_Pin);

/*!
 * @brief Writes the given value to the GPIO output
 *
 * @param  GPIOx: where x can be (A..E and H)
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @param [IN] value New GPIO output value
 * @retval none
 */
static inline void HW_GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t value)
{
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, (GPIO_PinState)value);
}

/*!
 * @brief Toggle the given GPIO output
 *
 * @param  GPIOx: where x can be (A..E and H)
 * @param  GPIO_Pin: specifies the port bit to be toggled.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @retval none
 */
static inline void HW_GPIO_Toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}

/*!
 * @brief Reads the current GPIO input value
 *
 * @param  GPIOx: where x can be (A..E and H)
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @retval value   Current GPIO input value
 */
static inline uint32_t HW_GPIO_Read(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
}

/**
 * Initialize all GPIO to analog input
 */
void HW_GpioDefaultInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __HW_GPIO_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
