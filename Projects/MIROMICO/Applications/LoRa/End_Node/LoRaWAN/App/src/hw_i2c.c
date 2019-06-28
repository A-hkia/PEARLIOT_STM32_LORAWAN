/*******************************************************************************
 * @file    hw_i2c.c
 * @author  Alex Raimondi
 * @version V1.0.2
 * @date    15-November-2016
 * @brief   I2C hardware functions
 ******************************************************************************
 */
#include "hw.h"
#include "hw_i2c.h"

static I2C_HandleTypeDef h_I2C;
PearlIot_i2c=&h_I2C;

void HW_I2C_MspInit(void);

void* HW_I2C1_GetHandle()
{
  return &h_I2C;
}

void HW_I2C_Init(void* handle)
{
  if (handle == &h_I2C) {
    h_I2C.Instance = I2C1;
  }
#if defined(STM32L151xBA)
  h_I2C.Init.ClockSpeed = 100000;
  h_I2C.Init.DutyCycle = I2C_DUTYCYCLE_2;
#elif defined(STM32L072xx) || defined(STM32L071xx)
  h_I2C.Init.Timing = 0x00B07DB9;
#else
#error Unkonwn controller
#endif
  h_I2C.Init.OwnAddress1 = 0;
  h_I2C.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  h_I2C.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  h_I2C.Init.OwnAddress2 = 0;
  h_I2C.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  h_I2C.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  HW_I2C_MspInit();
  if (HAL_I2C_Init(&h_I2C) != HAL_OK) {
    Error_Handler();
  }
}

/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HW_I2C_MspInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable I2C GPIO clocks */
  __GPIOB_CLK_ENABLE();

  /* I2C_EXPBD SCL and SDA pins configuration -------------------------------------*/
  GPIO_InitStruct.Pin        = GPIO_PIN_6 | GPIO_PIN_9;
  GPIO_InitStruct.Mode       = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed 	 = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Alternate  = GPIO_AF1_I2C1;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct );

  /* Enable the I2C_EXPBD peripheral clock */
  __I2C1_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  __HAL_RCC_I2C1_FORCE_RESET();
  /* Release the I2C peripheral clock reset */
  __HAL_RCC_I2C1_RELEASE_RESET();

  /* Enable and set I2C_EXPBD Interrupt to the highest priority */
  HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_IRQn);
}
void HW_I2C_DeInit(void* handle)
{
  HAL_I2C_DeInit(&h_I2C);
}

void HW_I2C_ReInit(void* handle)
{
  HW_I2C_DeInit(handle);
  HW_I2C_Init(handle);
}
