#include "hw.h"
#include "FMLR72_L0.h"
#include "hw_adc.h"
#include "hw_gpio.h"

/* Internal voltage reference, parameter VREFINT_CAL*/
#define VREFINT_CAL               ((uint16_t*) ((uint32_t) 0x1FF80078))

/* Internal temperature sensor: constants data used for indicative values in  */
/* this example. Refer to device datasheet for min/typ/max values.            */

/* Internal temperature sensor, parameter TS_CAL1: TS ADC raw data acquired at
 *a temperature of 110 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV). */
#define TEMP30_CAL_ADDR           ((uint16_t*) ((uint32_t) 0x1FF8007A))

/* Internal temperature sensor, parameter TS_CAL2: TS ADC raw data acquired at
 *a temperature of  30 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV). */
#define TEMP110_CAL_ADDR          ((uint16_t*) ((uint32_t) 0x1FF8007E))

#define INT_TEMP_OFFSET    3.5f

/* Vdda value with which temperature sensor has been calibrated in production
   (+-10 mV). */
#define VDDA_TEMP_CAL             ((uint32_t) 3000)
#define VDDA_VREFINT_CAL          ((uint32_t) 3000)

#define LORAWAN_MAX_BAT   254

/** Definition of adc range */
#define ADC_RANGE         4096

/* Battery level settings */
static battery_settings_t batteryConfig = {
  .vdd_vrefint = VDDA_VREFINT_CAL,
  .vdd_battery = VDD_BAT,
  .vdd_minimum = VDD_MIN,
};

/**
  * @brief Read voltage on ADC pin
  *
  * Read ADC value from port and compensate with calibrated VREF
  *
  * @param ADC channel
  * @retval Voltage on port in [mV]
  */
float HW_GetVoltage(uint32_t channel)
{
  uint16_t battLevel = HW_AdcReadChannel(ADC_CHANNEL_VREFINT);

  if (battLevel == 0) {
    return 0.0f;
  }

  float batteryLevelmV = (((float)VDDA_VREFINT_CAL * (*VREFINT_CAL)) / battLevel);

  uint16_t portLevel = HW_AdcReadChannel(channel);
  return (batteryLevelmV * portLevel) / ADC_RANGE;
}

int16_t HW_GetTemperatureLevel(void)
{
  uint16_t battLevel = HW_AdcReadChannel(ADC_CHANNEL_VREFINT);
  float batteryLevelmV = (((float)VDDA_VREFINT_CAL * (*VREFINT_CAL)) / battLevel);

  uint16_t tempLevel = HW_AdcReadChannel(ADC_CHANNEL_TEMPSENSOR);

  float tempVal = batteryLevelmV * tempLevel / (float)VDDA_VREFINT_CAL;
  tempVal = 80.0f / ((*TEMP110_CAL_ADDR) - (*TEMP30_CAL_ADDR)) * (tempVal - (*TEMP30_CAL_ADDR)) + 30.0f - INT_TEMP_OFFSET;

  return (int16_t)(tempVal * 10.0f + 0.5f);
}

/**
  * @brief This function returns the battery level based on VIN
  * @param none
  * @retval the battery level  1 (very low) to 254 (fully charged)
  */
uint8_t HW_GetVCC(uint32_t channel, float factor, bool hasEnPin , GPIO_TypeDef* enable_port, uint16_t enable_pin)
{
  uint8_t batteryLevel = 0;
  float measuredLevel = 0;
  uint32_t batteryLevelmV;

  if (hasEnPin) {
    HW_GPIO_Write(enable_port, enable_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    measuredLevel = HW_GetVoltage(channel);
    HW_GPIO_Write(enable_port, enable_pin, GPIO_PIN_RESET);
  }

  batteryLevelmV = (uint32_t)(measuredLevel * factor);

  if (batteryLevelmV > batteryConfig.vdd_battery) {
    batteryLevel = LORAWAN_MAX_BAT;
  } else if (batteryLevelmV < batteryConfig.vdd_minimum) {
    batteryLevel = 0;
  } else {
    batteryLevel = (((uint32_t)(batteryLevelmV - batteryConfig.vdd_minimum) * LORAWAN_MAX_BAT) / (batteryConfig.vdd_battery - batteryConfig.vdd_minimum));
  }
  return batteryLevel;
}

/**
  * @brief This function return the battery level
  * @param none
  * @retval the battery level  1 (very low) to 254 (fully charged)
  */
uint8_t HW_GetBatteryLevel(void)
{
  uint8_t batteryLevel = 0;
  uint32_t batteryLevelmV;

  batteryLevelmV = HW_GetBatteryLevel_mv();

  if (batteryLevelmV > VDD_BAT) {
    batteryLevel = LORAWAN_MAX_BAT;
  } else if (batteryLevelmV < VDD_MIN) {
    batteryLevel = 0;
  } else {
    batteryLevel = (((uint32_t)(batteryLevelmV - VDD_MIN) * LORAWAN_MAX_BAT) / (VDD_BAT - VDD_MIN));
  }
  return batteryLevel;
}

uint32_t HW_GetBatteryLevel_mv()
{
  uint16_t measuredLevel = HW_AdcReadChannel(ADC_CHANNEL_VREFINT);

  if (measuredLevel == 0) {
    return 0;
  } else {
    return (((uint32_t) VDDA_VREFINT_CAL * (*VREFINT_CAL)) / measuredLevel);
  }
}

/* Functions for setting battery thershold voltage etc. */
void HW_SetBatVrefint(uint32_t vref)
{
  batteryConfig.vdd_vrefint = vref;
}

void HW_SetBatVDD(uint32_t vddbat)
{
  batteryConfig.vdd_battery = vddbat;
}

void HW_SetBatVDDmin(uint32_t vddmin)
{
  batteryConfig.vdd_minimum = vddmin;
}
