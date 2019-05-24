#include <stdint.h>
#include "hw.h"
#include "sensors.h"


/** Cayenne LPP definitions */
#define LPP_DIGITAL_INPUT       0       // 1 byte
#define LPP_DIGITAL_OUTPUT      1       // 1 byte
#define LPP_ANALOG_INPUT        2       // 2 bytes, 0.01 signed
#define LPP_ANALOG_OUTPUT       3       // 2 bytes, 0.01 signed
#define LPP_LUMINOSITY          101     // 2 bytes, 1 lux unsigned
#define LPP_PRESENCE            102     // 1 byte, 1
#define LPP_TEMPERATURE         103     // 2 bytes, 0.1�C signed
#define LPP_RELATIVE_HUMIDITY   104     // 1 byte, 0.5% unsigned
#define LPP_ACCELEROMETER       113     // 2 bytes per axis, 0.001G
#define LPP_BAROMETRIC_PRESSURE 115     // 2 bytes 0.1 hPa Unsigned
#define LPP_GYROMETER           134     // 2 bytes per axis, 0.01 �/s
#define LPP_GPS                 136     // 3 byte lon/lat 0.0001 �, 3 bytes alt 0.01m

// Data ID + Data Type + Data Size
#define LPP_DIGITAL_INPUT_SIZE       3
#define LPP_DIGITAL_OUTPUT_SIZE      3
#define LPP_ANALOG_INPUT_SIZE        4
#define LPP_ANALOG_OUTPUT_SIZE       4
#define LPP_LUMINOSITY_SIZE          4
#define LPP_PRESENCE_SIZE            3
#define LPP_TEMPERATURE_SIZE         4
#define LPP_RELATIVE_HUMIDITY_SIZE   3
#define LPP_ACCELEROMETER_SIZE       8
#define LPP_BAROMETRIC_PRESSURE_SIZE 4
#define LPP_GYROMETER_SIZE           8
#define LPP_GPS_SIZE                 11

/**
 * Append temperature to LPP payload
 *
 * @param nr    Sensor number
 * @param t     Temperature value
 * @param buf   payload buffer
 * @param n     remaining bytes in payload buffer
 *
 * @return Number of bytes appended or -1 if no space left
 */
int8_t LPP_Temperature(uint8_t nr, int16_t t, uint8_t* buf, uint8_t* n)
{
  if (*n >= LPP_TEMPERATURE_SIZE) {
    PRINTF("SensNr %d: temperature=%.1f degC" NL, nr,  t / 10.0f);
    buf[0] = nr;
    buf[1] = LPP_TEMPERATURE;
    buf[2] = (t >> 8) & 0xFF;
    buf[3] = t & 0xFF;
    *n -= LPP_TEMPERATURE_SIZE;
    return LPP_TEMPERATURE_SIZE;
  } else {
    return -1;
  }
}

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
int8_t LPP_Humidity(uint8_t nr, uint8_t rh, uint8_t* buf, uint8_t* n)
{
  if (*n >= LPP_RELATIVE_HUMIDITY_SIZE) {
    PRINTF("SensNr %d: humidity=%.1f %%" NL, nr, rh / 2.0f);
    buf[0] = nr;
    buf[1] = LPP_RELATIVE_HUMIDITY;
    buf[2] = rh & 0xFF;
    *n -= LPP_RELATIVE_HUMIDITY_SIZE;
    return LPP_RELATIVE_HUMIDITY_SIZE;
  } else {
    return -1;
  }
}

/**
 * Append pressure to LPP payload
 *
 * @param nr    Sensor number
 * @param p     pressure value
 * @param buf   payload buffer
 * @param n     remaining bytes in payload buffer
 *
 * @return Number of bytes appended or -1 if no space left
 */
int8_t LPP_Pressure(uint8_t nr, uint16_t p, uint8_t* buf, uint8_t* n)
{
  if (*n >= LPP_BAROMETRIC_PRESSURE_SIZE) {
    PRINTF("SensNr %d: pressure=%.1f mbar" NL, nr, p / 10.f);
    buf[0] = nr;
    buf[1] = LPP_BAROMETRIC_PRESSURE;
    buf[2] = (p >> 8) & 0xFF;
    buf[3] = p & 0xFF;
    *n -= LPP_BAROMETRIC_PRESSURE_SIZE;
    return LPP_BAROMETRIC_PRESSURE_SIZE;
  } else {
    return -1;
  }
}

int8_t LPP_Distance(uint8_t nr, int16_t d, uint8_t* buf, uint8_t* n)
{
  // no direct support for distance => do it as analog
  if (*n >= LPP_ANALOG_INPUT_SIZE) {
    PRINTF("SensNr %d: distance=%u mm" NL, nr, d);
    buf[0] = nr;
    buf[1] = LPP_ANALOG_INPUT;
    buf[2] = (d >> 8) & 0xFF;
    buf[3] = d & 0xFF;
    *n -= LPP_ANALOG_INPUT_SIZE;
    return LPP_ANALOG_INPUT_SIZE;
  } else {
    return -1;
  }
}


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
int8_t LPP_Analog(uint8_t nr, int16_t a, uint8_t* buf, uint8_t* n)
{
  if (*n >= LPP_ANALOG_INPUT_SIZE) {
    PRINTF("SensNr %d: analog=%.2f" NL, nr, a / 100.f);
    buf[0] = nr;
    buf[1] = LPP_ANALOG_INPUT;
    buf[2] = (a >> 8) & 0xFF;
    buf[3] = a & 0xFF;
    *n -= LPP_ANALOG_INPUT_SIZE;
    return LPP_ANALOG_INPUT_SIZE;
  } else {
    return -1;
  }
}

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
int8_t LPP_Digital(uint8_t nr, uint8_t d, uint8_t* buf, uint8_t* n)
{
  if (*n >= LPP_DIGITAL_INPUT_SIZE) {
    PRINTF("SensNr %d: digital=0x%X" NL, nr, d);
    buf[0] = nr;
    buf[1] = LPP_DIGITAL_INPUT;
    buf[2] = d;
    *n -= LPP_DIGITAL_INPUT_SIZE;
    return LPP_DIGITAL_INPUT_SIZE;
  } else {
    return -1;
  }
}
