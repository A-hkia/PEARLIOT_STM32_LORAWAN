/******************************************************************************
 * @file    bsp.c
 * @author  MCD Application Team
 * @version V1.1.1
 * @date    01-June-2017
 * @brief   manages the sensors on the application
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "hw.h"
#include "bsp.h"
#include "FMLR72_L0.h"

/* Private variables ---------------------------------------------------------*/
static SHT_HandleTypedef h_sht;

/** Array of sensors */
static sensor_t sensors[MAX_SENSORS];

static I2C_HandleTypeDef h_I2C;

/** Definition of board LEDs */
GPIO_TypeDef* LED_PORT[LEDn]; // = {LED1_GPIO_PORT, LED2_GPIO_PORT, LED3_GPIO_PORT};
const uint16_t LED_PIN[LEDn]; // = {LED1_PIN, LED2_PIN, LED3_PIN};
const GPIO_PinState LED_OFF_STATE[LEDn]; // = {LED1_OFF, LED2_OFF, LED3_OFF};

static void I2C_Init()
{
  h_I2C.Instance = I2C1;
  h_I2C.Init.Timing = 0x00B07DB9;
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

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if (i2cHandle->Instance == I2C1) {
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

    GPIO_InitStruct.Alternate = I2C1_SCL_AF;
    HW_GPIO_Init(I2C1_SCL_PORT, I2C1_SCL_PIN, &GPIO_InitStruct);

    GPIO_InitStruct.Alternate = I2C1_SDA_AF;
    HW_GPIO_Init(I2C1_SDA_PORT, I2C1_SDA_PIN, &GPIO_InitStruct);

    __HAL_RCC_I2C1_CLK_ENABLE();
  }
}

const sensor_t* BSP_sensor_Read()
{
  for (uint8_t i = 0; i < MAX_SENSORS; i++) {
    switch (sensors[i].type) {
    case eNoSensor:
      return sensors;
    case eTempInt:
      sensors[i].data.temperature.t = HW_GetTemperatureLevel();
      break;
    case eSHT21:
      sensors[i].data.humidity.t = (SHTGetTemp((SHT_HandleTypedef*)(sensors[i].private)) + 5) / 10;
      sensors[i].data.humidity.rh = SHTGetHumidity((SHT_HandleTypedef*)(sensors[i].private));
      break;
    case eSiLab:
      sensors[i].data.humidity.t = (SHTGetTemp((SHT_HandleTypedef*)(sensors[i].private)) + 5) / 10;
      sensors[i].data.humidity.rh = SHTGetHumidity((SHT_HandleTypedef*)(sensors[i].private));
      break;
    case eSTS21:
      sensors[i].data.temperature.t = (SHTGetTemp((SHT_HandleTypedef*)(sensors[i].private)) + 5) / 10;
      break;
    default:
      break;
    }
  }
  return sensors;
}

/**
 * @brief  initializes the sensor
 *
 * @param sensorConfig  sensor configuration data
 * @retval Number of sensors found or -1 in case of error
 */
int8_t BSP_sensor_Init(const volatile eSensorConfig_t* sensorConfig)
{
  // Initialize I2C
  I2C_Init();

  uint8_t sensorCnt = 0;

  if (sensorConfig->reportBattLevel == ENABLE) {
    PRINTF("Battery level reporting enabled!" NL);
  }

  // check all possible sensors
  if (sensorConfig->internalTemp == ENABLE) {
    if (sensorCnt >= MAX_SENSORS) { goto to_many; }
    sensors[sensorCnt++].type = eTempInt;
    PRINTF("Internal temperature sensor enabled!" NL);
  }

  eSensorType_t sht = SHTInitSensor(&h_sht, &h_I2C);
  if (sht != eNoSensor) {
    if (sensorCnt >= MAX_SENSORS) { goto to_many; }
    sensors[sensorCnt].type = sht;
    sensors[sensorCnt++].private = &h_sht;
    if (sht == eSiLab) {
    	PRINTF("Sensor '%s' detected!\n\r", "SiLab");
		} else {
    	PRINTF("Sensor '%s' detected!\n\r", sht == eSHT21 ? "SHT21" : "STS21");
    }
  }
  return sensorCnt;

to_many:
  PRINTF("Too many sensors defined!" NL);
  return -1;
}
