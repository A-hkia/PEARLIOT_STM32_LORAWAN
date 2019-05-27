/*******************************************************************************
 * @file    stm32l1xx_hw.c
 * @author  MCD Application Team
 * @version V1.0.2
 * @date    15-November-2016
 * @brief   system hardware driver
 ******************************************************************************
 */
#include "hw.h"
#include "radio.h"
#include "debug.h"
#include "bsp.h"
#include "FMLR72_L0.h"

/** Get CUP reset source */
static eResetSrc_t HW_GetResetSource();

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

/**
 * Application level hardware initialization
 */
__weak void application_hw_init()     {}

/**
 * Application level hardware deinitialization
 */
__weak void application_hw_deinit()   {}

/**
  * Application level hardware io initialization after wake up from low power
  */
__weak void application_hw_ioinit()   {}

/**
  * Application level hardware io deinitialization before going to low power mode
  */
__weak void application_hw_iodeinit() {}

/**
  * @brief This function initializes the hardware
  * @param None
  * @retval CPU reset source
  */
eResetSrc_t HW_Init()
{
  if (McuInitialized == false) {

    Radio.IoInit();

#ifdef USE_ADC
    HW_AdcInit();
#endif
#ifdef USE_SPI1
    HW_SPI1_Init();
#endif
#ifdef USE_SPI2
    HW_SPI2_Init();
#endif
#ifdef USE_I2C1
    HW_I2C_Init(HW_I2C1_GetHandle());
#endif
#ifdef USE_RTC
    HW_RTC_Init();
#endif
#ifdef USE_USART1
    HW_UART_Init(HW_UART1_GetHandle(), 115200);
#endif
#ifdef USE_USART2
    HW_UART_Init(HW_UART2_GetHandle(), 9600);
#endif

    application_hw_init();
    HW_LED_Init(LED_GREEN);
    HW_LED_Init(LED_RED);

    McuInitialized = true;
  }
  return HW_GetResetSource();
}

/**
  * @brief This function Deinitializes the hardware
  * @param None
  * @retval None
  */
void HW_DeInit(void)
{
  Radio.IoDeInit();

#ifdef USE_SPI1
  HW_SPI1_DeInit();
#endif
#ifdef USE_SPI2
  HW_SPI2_DeInit();
#endif
#ifdef USE_USART1
  HW_UART_DeInit(HW_UART1_GetHandle());
#endif
#ifdef USE_USART2
  HW_UART_DeInit(HW_UART2_GetHandle());
#endif

  application_hw_deinit();
  HW_LED_DeInit(LED_GREEN);
  HW_LED_DeInit(LED_RED);

  McuInitialized = false;
}

/**
  * @brief This function Initializes the hardware Ios
  * @param None
  * @retval None
  */
void HW_IoInit(void)
{
  Radio.IoInit();

#ifdef USE_SPI1
  HW_SPI1_IoInit();
#endif
#ifdef USE_SPI2
  HW_SPI2_IoInit();
#endif
#ifdef USE_USART1
  HW_UART_IoInit(HW_UART1_GetHandle());
#endif
#ifdef USE_USART2
  HW_UART_IoInit(HW_UART2_GetHandle());
#endif
#ifdef USE_I2C1
  HW_I2C_IoInit(HW_I2C1_GetHandle());
#endif

  application_hw_ioinit();
  HW_LED_Init(LED_GREEN);
  HW_LED_Init(LED_RED);
}

/**
  * @brief This function Deinitializes the hardware Ios
  * @param None
  * @retval None
  */
void HW_IoDeInit(void)
{
  Radio.IoDeInit();
#ifdef USE_SPI1
  HW_SPI1_IoDeInit();
#endif
#ifdef USE_SPI2
  HW_SPI2_IoDeInit();
#endif
#ifdef USE_USART1
  HW_UART_IoDeInit(HW_UART1_GetHandle());
#endif
#ifdef USE_USART2
  HW_UART_IoDeInit(HW_UART2_GetHandle());
#endif
#ifdef USE_I2C1
  HW_I2C_IoDeInit(HW_I2C1_GetHandle());
#endif

  application_hw_iodeinit();
  HW_LED_DeInit(LED_GREEN);
  HW_LED_DeInit(LED_RED);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLLMUL                         = 6
  *            PLLDIV                         = 3
  *            Flash Latency(WS)              = 1
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable HSI Oscillator and Activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState            = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLL_DIV3;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* Set Voltage scale1 as MCU will run at 32MHz */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
}

static eResetSrc_t HW_GetResetSource()
{
  /* Check reset cause */
  if (__HAL_PWR_GET_FLAG(PWR_FLAG_WU)) {
    return eResetWU;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
    return eResetLPW;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
    return eResetSW;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
    return eResetPOR;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
    return eResetIWDG;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
    return eResetWWDG;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
    return eResetPIN;
  }

  return eResetNone;
}
