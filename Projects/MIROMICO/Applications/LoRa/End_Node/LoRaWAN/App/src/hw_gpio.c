/******************************************************************************
 * @file    hw_gpio.c
 * @author  MCD Application Team
 * @version V1.0.2
 * @date    15-November-2016
 * @brief   driver for GPIO
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "hw_gpio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static GpioIrqHandler* GpioIrq[16] = { NULL };

/* Private function prototypes -----------------------------------------------*/

static uint8_t HW_GPIO_GetBitPos(uint16_t GPIO_Pin);

#if defined(STM32L151xBA)
#error TBD
#elif defined(STM32L072xx) || defined(STM32L071xx)
GPIO_TypeDef * const port_list[NUM_PORTS] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, 0, 0, GPIOH };
#else
#error Unkonwn controller
#endif

/* Exported functions ---------------------------------------------------------*/

/*!
 * @brief Records the interrupt handler for the GPIO object
 *
 * @param  GPIOx: where x can be (A..E and H)
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @param [IN] prio       NVIC priority (0 is highest)
 * @param [IN] irqHandler  points to the  function to execute
 * @retval none
 */
void HW_GPIO_SetIrq(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t prio, GpioIrqHandler* irqHandler)
{
  IRQn_Type IRQnb;

  uint32_t BitPos = HW_GPIO_GetBitPos(GPIO_Pin) ;

  if (irqHandler != NULL) {
    GpioIrq[BitPos] = irqHandler;

    IRQnb = MSP_GetIRQn(GPIO_Pin);

    HAL_NVIC_SetPriority(IRQnb , prio, 0);
    HAL_NVIC_EnableIRQ(IRQnb);
  } else {
    GpioIrq[BitPos] = NULL;
  }
}

/*!
 * @brief Execute the interrupt from the object
 *
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @retval none
 */
void HW_GPIO_IrqHandler(uint16_t GPIO_Pin)
{
  uint32_t BitPos = HW_GPIO_GetBitPos(GPIO_Pin);

  if (GpioIrq[ BitPos ]  != NULL) {
    GpioIrq[ BitPos ]();
  }
}

/* Private functions ---------------------------------------------------------*/

/*!
 * @brief Get the position of the bit set in the GPIO_Pin
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @retval the position of the bit
 */
static uint8_t HW_GPIO_GetBitPos(uint16_t GPIO_Pin)
{
  uint8_t PinPos = 0;

  if ((GPIO_Pin & 0xFF00) != 0) { PinPos |= 0x8; }
  if ((GPIO_Pin & 0xF0F0) != 0) { PinPos |= 0x4; }
  if ((GPIO_Pin & 0xCCCC) != 0) { PinPos |= 0x2; }
  if ((GPIO_Pin & 0xAAAA) != 0) { PinPos |= 0x1; }

  return PinPos;
}
/**
  * @brief  Configure IO ports.
  * @retval None
  */
void HW_GpioDefaultInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct={0};

  /* Configure all GPIO as analog to reduce current consumption on non used IOs */
  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  /* All GPIOs except debug pins (SWCLK and SWD) */
  GPIO_InitStruct.Pin = GPIO_PIN_All & (~( GPIO_PIN_13 | GPIO_PIN_14) );
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* All GPIOs */
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();
}

