#include "sht_sts21.h"
#include "hw.h"
#include "hw_rtc.h"
#include "assert.h"
#include "debug.h"

#define SHT_READ_TEMP_NOHOLD 	0xF3
#define SHT_READ_TEMP_HOLD   	0xE3
#define SHT_READ_HUM_NOHOLD 	0xF5
#define SHT_READ_HUM_HOLD   	0xE5
#define SHT_READ_USER_REG		0xE7
#define SHT_WRITE_USER_REG		0xE6
#define SHT_SOFT_RESET			0xFE

#define SHT_TEMP_FLAG 			0x00
#define SHT_RH_FLAG				0x02
#define SI7021_READ_HEATER_REG	0x11

#define POLYNOMIAL 0x131 //P(x)=x^8+x^5+x^4+1 = 100110001
#define HW_I2C_Master_Transmit HAL_I2C_Master_Transmit_IT
#define HW_I2C_Master_Receive HAL_I2C_Master_Receive_IT
#define HW_RTC_GetTimerValueMs HW_RTC_GetTimerValue

/**
 * Check CRC
 *
 * @param	data	input data
 * @param	n		number of bytes in input data
 * @param	chck	checksum to match
 * @return	0 on match, -1 on fail
 */
static int8_t SHTCheckCrc(uint8_t data[], uint8_t n, uint8_t chck)
{
  uint8_t crc = 0;
  uint8_t byteCtr;
//calculates 8-Bit chck with given polynomial
  for (byteCtr = 0; byteCtr < n; ++byteCtr) {
    crc ^= (data[byteCtr]);
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) { crc = (crc << 1) ^ POLYNOMIAL; }
      else { crc = (crc << 1); }
    }
  }
  return (crc == chck) ? 0 : -1;
}

/**
 * Read out data from sensor
 *
 * Read out data using the hold commands. 3 bytes are read and
 * checksum is checked
 *
 * @param	i2c		handle of the i2c bus
 * @param	addr	i2c address of the sensor
 * @param	cmd		Temp or humidity read out command
 *
 * @retval	data	data from sensor
 * @return	0 in case of success, -1 otherwise
 */
static int8_t SHTGetValue(SHT_HandleTypedef* handle, uint8_t cmd, uint16_t* data)
{
  uint8_t buf[3];
  buf[0] = cmd;

  assert(data != 0);
  if (handle->i2caddress == 0) { return -1; }

  //  just writes to the I2C
  HW_I2C_Master_Transmit(handle->hi2c, handle->i2caddress, (uint8_t*)&buf, 1);
  uint32_t tickstart = HW_RTC_GetTimerValueMs();

  while (1) {
    HAL_Delay(5);
    // stop after successful reading
    if (HW_I2C_Master_Receive(handle->hi2c, handle->i2caddress, (uint8_t*)&buf, 3)) {
      break;
    }
    if ((HW_RTC_GetTimerValueMs() - tickstart) >= 85) {
      return -1;
    }
  }

  if (SHTCheckCrc(buf, 2, buf[2]) >= 0) {
    *data = (buf[0] << 8 | buf[1]);
    return 0;
  } else {
    return -1;
  }
}

eSensorType_t SHTInitSensor(SHT_HandleTypedef* handle, void* hi2c)
{
  handle->hi2c = hi2c;

  handle->i2caddress = I2C_ADDR_SHT21;
  int16_t t = SHTGetTemp(handle);
  int8_t silab=0;
  uint16_t dataTemp;

  //Check if sensor SHT or Silab-> Ack=Silab, Nack=SHT
  silab = SHTGetValue(handle, SI7021_READ_HEATER_REG, &dataTemp);

  if (t != TEMP_INV) {
	if (silab == 0) {
		return eSiLab;
	} else {
		// SHT21 connected
		return eSHT21;
	}
  }
  handle->i2caddress = I2C_ADDR_STS21;
  t = SHTGetTemp(handle);
  if (t > TEMP_INV) {
    // STS21 connected
    return eSTS21;
  }
  // failed
  handle->i2caddress = 0;
  return eNoSensor;
}

int16_t SHTGetTemp(SHT_HandleTypedef* handle)
{
  uint16_t value;
  if (SHTGetValue(handle, SHT_READ_TEMP_NOHOLD, &value) >= 0) {
    if ((value & 0x02) == SHT_TEMP_FLAG) {
      value &= 0xfffc;
      // Formula is T = -46.85 + 175.72 * value / 2^16
      // computing in fixed point
      return (((-4685 * 16384) + 4393 * (int32_t)value + 8192) / 16384);
    }
  }
  // something went wrong
  return TEMP_INV;
}

uint8_t SHTGetHumidity(SHT_HandleTypedef* handle)
{
  uint16_t value;
  if (SHTGetValue(handle, SHT_READ_HUM_NOHOLD, &value) >= 0) {
    if ((value & 0x02) == SHT_RH_FLAG) {
      value &= 0xfffc;
      // Formula is rH = 6 + 125 * value / 2^16
      // computing in fixed point
      return (((-12 * 32768) + 125 * (uint32_t)value + 16384) / 32768);
    }
  }
// something wet wrong
  return RH_INV;
}
