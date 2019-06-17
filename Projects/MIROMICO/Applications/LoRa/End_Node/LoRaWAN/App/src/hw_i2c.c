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
  if (HAL_I2C_Init(&h_I2C) != HAL_OK) {
    Error_Handler();
  }
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
