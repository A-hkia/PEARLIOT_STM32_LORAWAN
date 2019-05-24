/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_H__
#define __BSP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <sensors.h>

/** maximum number of sensors */
#define MAX_SENSORS 3

/** Device sensor configuration data */
typedef struct {
  FunctionalState reportBattLevel;  //< Flag to set if battery level is included in payload
  FunctionalState internalTemp;     //< Flag to set if internal temperature sensor is included in payload
  int16_t upperThreshold;
  int16_t lowerThreshold;
  int16_t hysteresis;
} eSensorConfig_t;

/**
 * @brief  initialises the sensor
 *
 * @param sensorConfig  sensor configuration data
 * @retval None
 */
int8_t BSP_sensor_Init(const volatile eSensorConfig_t* sensorConfig);

/**
 * @brief  sensor  read.
 *
 * @retval sensor_data
 */
const sensor_t* BSP_sensor_Read();

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H__ */
