/*
 ******************************************************************************
 * @file    hw_msp.h
 * @author  MCD Application Team
 * @version V1.0.2
 * @date    15-November-2016
 * @brief   Header for driver hw msp module
 ******************************************************************************
 */
#ifndef __HW_I2C_H__
#define __HW_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/** Get handle of I2C1 bus */
void* HW_I2C1_GetHandle();

/** Initialize bus with given handle */
void HW_I2C_Init(void* handle);

/** Deinitialize bus with given handle */
void HW_I2C_DeInit(void* handle);

void HW_I2C_ReInit(void* handle);

static inline bool HW_I2C_Master_Transmit(void* handle, uint16_t addr, uint8_t* buffer, uint16_t size)
{
  return HAL_I2C_Master_Transmit((I2C_HandleTypeDef*)handle, addr, buffer, size, 100) == HAL_OK;
}

static inline bool HW_I2C_Master_Receive(void* handle, uint16_t addr, uint8_t* buffer, uint16_t size)
{
  return HAL_I2C_Master_Receive(handle, addr, buffer, size, 1000) == HAL_OK;
}

static inline bool HW_I2C_Mem_Write(I2C_HandleTypeDef* hi2c, uint16_t address, uint16_t reg, uint16_t MemAddSize, uint8_t* pdata, uint16_t count)
{
  return HAL_I2C_Mem_Write(hi2c, address, reg, MemAddSize, pdata, count, 1000) == HAL_OK;
}

static inline bool HW_I2C_Mem_Read(I2C_HandleTypeDef* hi2c, uint16_t address, uint16_t reg, uint16_t MemAddSize, uint8_t* pdata, uint16_t count)
{
  return HAL_I2C_Mem_Read(hi2c, address, reg, MemAddSize, pdata, count, 1000) == HAL_OK;
}

static inline void HW_I2C_IoInit(void* handle)
{
  __HAL_I2C_ENABLE((I2C_HandleTypeDef*)handle);
}

static inline void HW_I2C_IoDeInit(void* handle)
{
  __HAL_I2C_DISABLE((I2C_HandleTypeDef*)handle);
}

#ifdef __cplusplus
}
#endif

#endif
