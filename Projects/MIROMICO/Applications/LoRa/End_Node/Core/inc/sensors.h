/******************************************************************************
 * @file    sensors.h
 * @author  MCD Application Team
 * @version V1.0.2
 * @date    15-November-2016
 * @brief   contains sensor types
 ******************************************************************************
 */
#ifndef __SENSORS_H__
#define __SENSORS_H__

#ifdef __cplusplus
extern "C" {
#endif

/** Define for new line character(s) */
#define NL "\n\r"

/** Invalid temperature value in case of read out error */
#define TEMP_INV  30000

/** Invalid rH value in case of read out error */
#define RH_INV  0xFF

/** Invalid pressure value in case of read out error */
#define PRESSURE_INV  0

/** Enum for all known sensor types */
typedef enum {
  eNoSensor,  ///< No sensor
  eTempInt,   ///< internal temperature sensor
  eSHT21,     ///< Sensirion xxxx temperature/humidity
  eSiLab,	  ///< Silabs xxxx temperature/humidity
  eSTS21,     ///< Sensirion xxxx temperature
  eTmp112,    ///< TI TMP112 temperature sensor
  eTmp112Ext, ///< TI TMP112 temperature sensor on external PCB
  eBMP280,    ///< Bosch ambient pressure and temperature
  eVL53L0X,   ///< ST ToF sensor
  eUS100,     ///< US-100 ultrasonic distance
  eAnalogIn,  ///< Analog input
  eDigitalIn, ///< Digital input
  eLis2dh,    ///< Motion sensor
  eDS18B20    ///< one wire temperature sensor
} eSensorType_t;

/** Temperature sensor */
typedef struct {
  int16_t t;  //< temperature in 0.1�C, signed
} eTemperatureSensor_t;

/** Humidity/temperature sensor */
typedef struct {
  int16_t t;    //< temperature in 0.1�C, signed
  uint8_t rh;   //< humidity in 0.5%
} eHumiditySensor_t;

/** Pressure/temperature sensor */
typedef struct {
  int16_t t;    //< temperature in 0.1�C, signed
  uint16_t p;    //< ambient pressure in 0.1 hPa
} ePressureSensor_t;

/** Distance sensor */
typedef struct {
  uint16_t d;   //< distance in mm
} eDistanceSensor_t;

/** Analog input */
typedef struct {
  int16_t ai;   //< analog value in 0.01, signed
} eAnalogInput_t;

/** Digital input */
typedef struct {
  uint8_t di;   //< digital input
} eDigitalInput_t;

/** Union of all sensor data types */
typedef union {
  eTemperatureSensor_t temperature; //< temperature sensor
  eHumiditySensor_t humidity;       //< humidity sensor
  ePressureSensor_t pressure;       //< ambient pressure sensor
  eDistanceSensor_t distance;       //< distance sensor
  eAnalogInput_t analog;            //< analog input
  eDigitalInput_t digital;          //< digital input
} eSensorData_t;

/** Sensor data type */
typedef struct {
  eSensorType_t type; //< Sensor type
  void* private;      //< Private sensor data (e.g. handle)
  eSensorData_t data; //< Sensor data
} sensor_t;

typedef union {
  uint64_t u64;
  uint32_t u32[2];
} oidData_t;

// Include project specific sensor configuration */
#include "sensor_conf.h"

/**
 * Append temperature to LPP payload
 *
 * @param nr    Sensor number
 * @param temp  Temperature value
 * @param buf   payload buffer
 * @param n     remaining bytes in payload buffer
 *
 * @return Number of bytes appended or -1 if no space left
 */
int8_t LPP_Temperature(uint8_t nr, int16_t temp, uint8_t* buf, uint8_t* n);

/**
 * Append humidity to LPP payload
 *
 * @param nr    Sensor number
 * @param rh    humidity value
 * @param buf   payload buffer
 * @param n     remaining bytes in payload buffer
 *
 * @return Number of bytes appended or -1 if no space left
 */
int8_t LPP_Humidity(uint8_t nr, uint8_t rh, uint8_t* buf, uint8_t* n);

/**
 * Append pressure to LPP payload
 *
 * @param nr    Sensor number
 * @param p     humidity value
 * @param buf   payload buffer
 * @param n     remaining bytes in payload buffer
 *
 * @return Number of bytes appended or -1 if no space left
 */
int8_t LPP_Pressure(uint8_t nr, uint16_t p, uint8_t* buf, uint8_t* n);

/**
 * Append distance value to LPP payload
 *
 * @param nr    Sensor number
 * @param d     distance value
 * @param buf   payload buffer
 * @param n     remaining bytes in payload buffer
 *
 * @return Number of bytes appended or -1 if no space left
 */
int8_t LPP_Distance(uint8_t nr, int16_t d, uint8_t* buf, uint8_t* n);

/**
 * Append analog value to LPP payload
 *
 * @param nr    Sensor number
 * @param a     analog value
 * @param buf   payload buffer
 * @param n     remaining bytes in payload buffer
 *
 * @return Number of bytes appended or -1 if no space left
 */
int8_t LPP_Analog(uint8_t nr, int16_t a, uint8_t* buf, uint8_t* n);

/**
 * Append digital input value to LPP payload
 *
 * @param nr    Sensor number
 * @param d     digital value
 * @param buf   payload buffer
 * @param n     remaining bytes in payload buffer
 *
 * @return Number of bytes appended or -1 if no space left
 */
int8_t LPP_Digital(uint8_t nr, uint8_t d, uint8_t* buf, uint8_t* n);

#ifdef __cplusplus
}
#endif

#endif /* __SENSORS_H__ */

