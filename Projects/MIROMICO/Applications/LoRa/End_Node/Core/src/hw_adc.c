#include "hw.h"
#include "FMLR72_L0.h"

/** Definition of adc resolution */
#define ADC_RESOLUTION    ADC_RESOLUTION_12B

static ADC_HandleTypeDef hadc;
/*!
 * Flag to indicate if the ADC is Initialized
 */
static bool AdcInitialized = false;

/**
  * @brief This function initializes the ADC
  * @param none
  * @retval none
  */
void HW_AdcInit(void)
{
  if (AdcInitialized == false) {
    AdcInitialized = true;
#if 0
    GPIO_InitTypeDef initStruct;
#endif

    hadc.Instance  = ADC1;

    hadc.Init.OversamplingMode      = DISABLE;

    hadc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc.Init.LowPowerAutoPowerOff  = DISABLE;
    hadc.Init.LowPowerFrequencyMode = ENABLE;
    hadc.Init.LowPowerAutoWait      = DISABLE;

    hadc.Init.Resolution            = ADC_RESOLUTION;
    hadc.Init.SamplingTime          = ADC_SAMPLETIME_160CYCLES_5;
    hadc.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc.Init.ContinuousConvMode    = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc.Init.DMAContinuousRequests = DISABLE;

    ADCCLK_ENABLE();

    HAL_ADC_Init(&hadc);
#if 0
    initStruct.Mode = GPIO_MODE_ANALOG;
    initStruct.Pull = GPIO_NOPULL;
    initStruct.Speed = GPIO_SPEED_HIGH;

    HW_GPIO_Init(BAT_LEVEL_PORT, BAT_LEVEL_PIN, &initStruct);
#endif
  }
}
/**
  * @brief This function De-initializes the ADC
  * @param none
  * @retval none
  */
void HW_AdcDeInit(void)
{
  AdcInitialized = false;
}

/**
  * @brief This function De-initializes the ADC
  * @param Channel
  * @retval Value
  */
uint16_t HW_AdcReadChannel(uint32_t Channel)
{

  ADC_ChannelConfTypeDef adcConf;
  uint16_t adcData = 0;

  if (AdcInitialized == true) {
    /* wait the the Vrefint used by adc is set */
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_VREFINTRDY) == RESET) {};

    ADCCLK_ENABLE();

    /*calibrate ADC if any calibration hardware*/
    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);

    /* Deselects all channels*/
    adcConf.Channel = ADC_CHANNEL_MASK;
    adcConf.Rank = ADC_RANK_NONE;
    HAL_ADC_ConfigChannel(&hadc, &adcConf);

    /* configure adc channel */
    adcConf.Channel = Channel;
    adcConf.Rank = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel(&hadc, &adcConf);

    /* Start the conversion process */
    HAL_ADC_Start(&hadc);

    /* Wait for the end of conversion */
    HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);

    /* Get the converted value of regular channel */
    adcData = HAL_ADC_GetValue(&hadc);

    __HAL_ADC_DISABLE(&hadc) ;
    ADCCLK_DISABLE();
  }
  return adcData;
}
