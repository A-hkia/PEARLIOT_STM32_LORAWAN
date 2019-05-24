#ifndef __STS_SHT21_H__
#define __STS_SHT21_H__

#include <stdint.h>
#include <hw.h>
#include "sensors.h"

/** SHT sensor device handle type */
typedef struct {
  uint8_t i2caddress; 				///< Is 0x76, meaning you need to write 0x76<<1 to it.
  void* hi2c; 			// The I2C Handle for STM32 HAL_Driver
} SHT_HandleTypedef;

/** I2C address of STS21 sensor */
#define I2C_ADDR_STS21	0x94
/** I2C address of SHT21 sensor */
#define I2C_ADDR_SHT21	0x80

/**
 * Initialize sensor
 *
 * Set up handler and initialize sensor.
 *
 * @param handle      Sensor device handle
 * @param hi2c        I2C bus handler to use in conjuction with this sensor
 *
 * @return  sensor i2c address on success, 0 on error
 */
eSensorType_t SHTInitSensor(SHT_HandleTypedef* handle, void* hi2c);

/**
 * Get temperature reading
 *
 * @param	handle	Sensor device handle
 *
 * @return	temperature value in 1/100 °C, value < -50 to flag an error
 */
int16_t SHTGetTemp(SHT_HandleTypedef* handle);

/**
 * Get humidity reading
 *
 * @param	handle	Sensor device handle
 *
 * @return	humidity value in 0.5 % rH, value > 200 to flag an error
 */
uint8_t SHTGetHumidity(SHT_HandleTypedef* handle);

#endif  // __STS_SHT21_H__
