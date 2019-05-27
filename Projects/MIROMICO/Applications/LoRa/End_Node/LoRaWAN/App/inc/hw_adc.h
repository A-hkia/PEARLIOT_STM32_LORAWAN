/******************************************************************************
 * @file    hw_msp.h
 * @author  MCD Application Team
 * @version V1.0.2
 * @date    15-November-2016
 * @brief   Header for driver hw msp module
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __HW_ADC_H__
#define __HW_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct {
  uint32_t vdd_vrefint;
  uint32_t vdd_battery;
  uint32_t vdd_minimum;
} battery_settings_t;
/* Exported constants --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/*!
 * \brief Get the current temperature from internal temperature sensor
 *
 * \retval value  temperature in 0.1 °C
 */
int16_t HW_GetTemperatureLevel(void);

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level ( 0: very low, 254: fully charged )
 */
uint8_t HW_GetBatteryLevel();

/*!
 * \brief Get the current battery level, based on external voltage
 *
 * \retval value  battery level ( 0: very low, 254: fully charged )
 */
uint8_t HW_GetVCC(uint32_t channel, float factor, bool hasEnPin , GPIO_TypeDef* enable_port, uint16_t enable_pin);

/*!
 * \brief Get the current battery level
 *
 * @return Battery voltage level in mv
 */
uint32_t HW_GetBatteryLevel_mv(void);

/**
  * @brief Read voltage on ADC pin
  *
  * Read ADC value from port and compensate with calibrated VREF
  *
  * @param ADC channel
  * @retval Voltage on port in [mV]
  */
float HW_GetVoltage(uint32_t channel);

/*!
 * \brief Initializes the ADC input
 *
 * \param [IN] scl  ADC input pin name to be used
 */
void HW_AdcInit(void);

/*!
 * \brief DeInitializes the ADC
 *
 * \param [IN] none
 */
void HW_AdcDeInit(void);

/*!
 * \brief Read the analogue voltage value
 *
 * \param [IN] Channel to read
 * \retval value    Analogue pin value
 */
uint16_t HW_AdcReadChannel(uint32_t Channel);

/* Functions for setting battery thershold voltage etc. */
void HW_SetBatVrefint(uint32_t vref);
void HW_SetBatVDD(uint32_t vddbat);
void HW_SetBatVDDmin(uint32_t vddmin);

#ifdef __cplusplus
}
#endif

#endif /* __HW_ADC_H__ */
