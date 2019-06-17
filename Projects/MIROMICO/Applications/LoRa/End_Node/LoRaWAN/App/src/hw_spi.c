/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "hw_gpio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/*!
 * @brief Calculates Spi Divisor based on Spi Frequency and Mcu Frequency
 *
 * @param [IN] Spi Frequency
 * @retval Spi divisor
 */
static uint32_t SpiFrequency(uint32_t hz);

/* Exported functions ---------------------------------------------------------*/

#if defined(USE_SPI1)
static SPI_HandleTypeDef hspi1;

SPI_HandleTypeDef* HW_SPI1_GetHandle()
{
  return &hspi1;
}

void HW_SPI1_IoDeInit(void)
{
  GPIO_InitTypeDef initStruct = {0};

  initStruct.Mode = GPIO_MODE_OUTPUT_PP;
  initStruct.Pull = GPIO_PULLDOWN;
  HW_GPIO_Init(SPI1_MOSI_PORT, SPI1_MOSI_PIN, &initStruct);
  HW_GPIO_Write(SPI1_MOSI_PORT, SPI1_MOSI_PIN, 0);
  HW_GPIO_Init(SPI1_MISO_PORT, SPI1_MISO_PIN, &initStruct);
  HW_GPIO_Write(SPI1_MISO_PORT, SPI1_MISO_PIN, 0);
  HW_GPIO_Init(SPI1_SCLK_PORT, SPI1_SCLK_PIN, &initStruct);
  HW_GPIO_Write(SPI1_SCLK_PORT, SPI1_SCLK_PIN, 0);
}

/*!
 * @brief De-initializes the SPI1 object and MCU peripheral
 *
 * @param [IN] none
 */
void HW_SPI1_DeInit(void)
{
  HAL_SPI_DeInit(&hspi1);

  /*##-1- Reset peripherals ####*/
  __HAL_RCC_SPI1_FORCE_RESET();
  __HAL_RCC_SPI1_RELEASE_RESET();
  /*##-2- Configure the SPI1 GPIOs */
  HW_SPI1_IoDeInit();
}

void HW_SPI1_IoInit(void)
{
  GPIO_InitTypeDef initStruct = {0};
  initStruct.Mode = GPIO_MODE_AF_PP;
  initStruct.Pull = GPIO_NOPULL;
  initStruct.Speed = GPIO_SPEED_HIGH;

  initStruct.Alternate = SPI1_SCLK_AF;
  HW_GPIO_Init(SPI1_SCLK_PORT, SPI1_SCLK_PIN, &initStruct);
  initStruct.Alternate = SPI1_MISO_AF;
  HW_GPIO_Init(SPI1_MISO_PORT, SPI1_MISO_PIN, &initStruct);
  initStruct.Alternate = SPI1_MOSI_AF;
  HW_GPIO_Init(SPI1_MOSI_PORT, SPI1_MOSI_PIN, &initStruct);
}

/*!
 * @brief Initializes the SPI1 object and MCU peripheral
 *
 * @param [IN] none
 */
void HW_SPI1_Init(void)
{
  /*##-1- Configure the SPI1 peripheral */
  /* Set the SPI1 parameters */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode           = SPI_MODE_MASTER;
  hspi1.Init.Direction      = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize       = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity    = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase       = SPI_PHASE_1EDGE;
  hspi1.Init.NSS            = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SpiFrequency(10000000);
  hspi1.Init.FirstBit       = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode         = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

  SPI1_CLK_ENABLE();

  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the SPI1 GPIOs */
  HW_SPI1_IoInit();
}

#endif

#if defined(USE_SPI2)
static SPI_HandleTypeDef hspi2;

SPI_HandleTypeDef* HW_SPI2_GetHandle()
{
  return &hspi2;
}

void HW_SPI2_IoDeInit(void)
{
  GPIO_InitTypeDef initStruct = {0};

  initStruct.Mode = GPIO_MODE_OUTPUT_PP;
  initStruct.Pull = GPIO_PULLDOWN;
  HW_GPIO_Init(SPI2_MOSI_PORT, SPI2_MOSI_PIN, &initStruct);
  HW_GPIO_Write(SPI2_MOSI_PORT, SPI2_MOSI_PIN, 0);
  HW_GPIO_Init(SPI2_MISO_PORT, SPI2_MISO_PIN, &initStruct);
  HW_GPIO_Write(SPI2_MISO_PORT, SPI2_MISO_PIN, 0);
  HW_GPIO_Init(SPI2_SCLK_PORT, SPI2_SCLK_PIN, &initStruct);
  HW_GPIO_Write(SPI2_SCLK_PORT, SPI2_SCLK_PIN, 0);
}

/*!
 * @brief De-initializes the SPI2 object and MCU peripheral
 *
 * @param [IN] none
 */
void HW_SPI2_DeInit(void)
{
  HAL_SPI_DeInit(&hspi2);

  /*##-1- Reset peripherals ####*/
  __HAL_RCC_SPI2_FORCE_RESET();
  __HAL_RCC_SPI2_RELEASE_RESET();
  /*##-2- Configure the SPI GPIOs */
  HW_SPI2_IoDeInit();
}

void HW_SPI2_IoInit(void)
{
  GPIO_InitTypeDef initStruct = {0};

  initStruct.Mode = GPIO_MODE_AF_PP;
  initStruct.Pull = GPIO_NOPULL;
  initStruct.Speed = GPIO_SPEED_HIGH;

  initStruct.Alternate = SPI2_SCLK_AF;
  HW_GPIO_Init(SPI2_SCLK_PORT, SPI2_SCLK_PIN, &initStruct);
  initStruct.Alternate = SPI2_MISO_AF;
  HW_GPIO_Init(SPI2_MISO_PORT, SPI2_MISO_PIN, &initStruct);
  initStruct.Alternate = SPI2_MOSI_AF;
  HW_GPIO_Init(SPI2_MOSI_PORT, SPI2_MOSI_PIN, &initStruct);
}


/*!
 * @brief Initializes the SPI2 object and MCU peripheral
 *
 * @param [IN] none
 */
void HW_SPI2_Init(void)
{
  /*##-1- Configure the SPI peripheral */
  /* Set the SPI parameters */
  hspi2.Instance = SPI2;

  hspi2.Init.BaudRatePrescaler = SpiFrequency(10000000);
  hspi2.Init.Direction      = SPI_DIRECTION_2LINES;
  hspi2.Init.Mode           = SPI_MODE_MASTER;
  hspi2.Init.CLKPolarity    = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase       = SPI_PHASE_1EDGE;
  hspi2.Init.DataSize       = SPI_DATASIZE_8BIT;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.FirstBit       = SPI_FIRSTBIT_MSB;
  hspi2.Init.NSS            = SPI_NSS_SOFT;
  hspi2.Init.TIMode         = SPI_TIMODE_DISABLE;

  SPI2_CLK_ENABLE();

  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the SPI GPIOs */
  HW_SPI2_IoInit();
}
#endif

void HW_SPI_Transmit(SPI_HandleTypeDef* hspi, uint8_t* txData, uint16_t size, uint32_t timeout)
{
  HAL_SPI_Transmit(hspi, txData, size, timeout);
}

void HW_SPI_Receive(SPI_HandleTypeDef* hspi, uint8_t* rxData, uint16_t size, uint32_t timeout)
{
  HAL_SPI_Receive(hspi, rxData, size, timeout);
}

/*!
 * @brief Sends outData and receives inData
 *
 * @param [IN] outData Byte to be sent
 * @retval inData      Received byte.
 */
uint16_t HW_SPI_InOut(SPI_HandleTypeDef* hspi, uint16_t txData)
{
  uint16_t rxData ;

  HAL_SPI_TransmitReceive(hspi, (uint8_t*) &txData, (uint8_t*) &rxData, 1, HAL_MAX_DELAY);

  return rxData;
}

/* Private functions ---------------------------------------------------------*/

static uint32_t SpiFrequency(uint32_t hz)
{
  uint32_t divisor = 0;
  uint32_t SysClkTmp = SystemCoreClock;
  uint32_t baudRate;

  while (SysClkTmp > hz) {
    divisor++;
    SysClkTmp = (SysClkTmp >> 1);

    if (divisor >= 7) {
      break;
    }
  }

  baudRate = (((divisor & 0x4) == 0) ? 0x0 : SPI_CR1_BR_2) |
             (((divisor & 0x2) == 0) ? 0x0 : SPI_CR1_BR_1) |
             (((divisor & 0x1) == 0) ? 0x0 : SPI_CR1_BR_0);

  return baudRate;
}
