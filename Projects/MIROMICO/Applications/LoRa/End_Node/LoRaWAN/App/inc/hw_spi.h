/******************************************************************************
 * @file    hw_spi.h
 * @author  MCD Application Team
 * @version V1.0.2
 * @date    15-November-2016
 * @brief   Header for driver hw_spi.c module
 ******************************************************************************
 ******************************************************************************
 */
#ifndef __HW_SPI_H__
#define __HW_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

SPI_HandleTypeDef* HW_SPI1_GetHandle();
SPI_HandleTypeDef* HW_SPI2_GetHandle();

/*!
 * @brief Initializes the SPI1 object and MCU peripheral
 *
 * @param [IN] none
 */
void HW_SPI1_Init(void);

/*!
 * @brief De-initializes the SPI1 object and MCU peripheral
 *
 * @param [IN] none
 */
void HW_SPI1_DeInit(void);

/*!
 * @brief Initializes the SPI1 IOs
 *
 * @param [IN] none
 */
void HW_SPI1_IoInit(void);

/*!
 * @brief De-initializes the SPI1 IOs
 *
 * @param [IN] none
 */
void HW_SPI1_IoDeInit(void);

/*!
 * @brief Initializes the SPI2 object and MCU peripheral
 *
 * @param [IN] none
 */
void HW_SPI2_Init(void);

/*!
 * @brief De-initializes the SPI2 object and MCU peripheral
 *
 * @param [IN] none
 */
void HW_SPI2_DeInit(void);

/*!
 * @brief Initializes the SPI2 IOs
 *
 * @param [IN] none
 */
void HW_SPI2_IoInit(void);

/*!
 * @brief De-initializes the SPI2 IOs
 *
 * @param [IN] none
 */
void HW_SPI2_IoDeInit(void);

/*!
 * @brief Sends outData and receives inData
 *
 * @param [IN] outData Byte to be sent
 * @retval inData      Received byte.
 */
void HW_SPI_Transmit(SPI_HandleTypeDef* hspi, uint8_t* txData, uint16_t size, uint32_t timeout);

/*!
 * @brief Sends outData and receives inData
 *
 * @param [IN] outData Byte to be sent
 * @retval inData      Received byte.
 */
void HW_SPI_Receive(SPI_HandleTypeDef* hspi, uint8_t* rxData, uint16_t size, uint32_t timeout);

/*!
 * @brief Sends outData and receives inData
 *
 * @param [IN] outData Byte to be sent
 * @retval inData      Received byte.
 */
uint16_t HW_SPI_InOut(SPI_HandleTypeDef* hspi, uint16_t txData);

#ifdef __cplusplus
}
#endif

#endif  /* __HW_SPI_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
