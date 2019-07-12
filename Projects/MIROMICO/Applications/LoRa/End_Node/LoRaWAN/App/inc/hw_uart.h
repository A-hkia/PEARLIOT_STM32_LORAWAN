/******************************************************************************
 * @file    HW_UART.h
 * @author  MCD Application Team
 * @version V1.0.2
 * @date    15-November-2016
 * @brief   Header for HW_UART.c module
 ******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_UART_H__
#define __HW_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/**
* @brief  Get UART handle for HW_UART2
* @param  None
* @return None
*/
void* HW_UART1_GetHandle();

/**
* @brief  Get UART handle for HW_UART2
* @param  None
* @return None
*/
void* HW_UART2_GetHandle();

/**
* @brief  Init the HW_UART1.
* @param  None
* @return None
*/
void HW_UART_Init(void* handle, uint32_t br);

/**
* @brief  DeInit the HW_UART1.
* @param  None
* @return None
*/
void HW_UART_DeInit(void* handle);

/**
* @brief  sends data in buffer on com port
* @param  uart handle
* @param  buffer
* @param  size
* @return None
*/
void HW_UART_SendBuffer(void* handle, const uint8_t* buffer, uint16_t size);

/**
* @brief  sends string on com port
* @param  string
* @return None
*/
void HW_UART_Send(void* handle, const char* format, ...);

/**
 *
 */
int8_t HW_UART_ReceiveBuffer(void* handle, uint8_t* buffer, uint16_t size, uint16_t timeout_ms);

/**
* @brief  Init the HW_UART1 IOs.
* @param  None
* @return None
*/
void HW_UART_IoInit(void* handle);

/**
* @brief  DeInit the HW_UART1 IOs.
* @param  None
* @return None
*/
void HW_UART_IoDeInit(void* handle);

/**
 * @brief  DeInit the VCOM RX
 * @param  None
 * @retval None
 */
void HW_UART_DeInitReceive(void* handle);

/**
 * @brief  Init the VCOM RX
 * @param  None
 * @retval None
 */
void HW_UART_ReceiveInit(void* handle, bool useInt);

/**
 * @brief  Flush UART receive register
 * @param  None
 * @retval None
 */
void HW_UART_FlushRx(void* handle);

/**
 * @brief  Checks if a new character has been received on com port
 * @param  None
 * @retval Returns SET if new character has been received on com port, RESET otherwise
 */
FlagStatus HW_UART_IsNewCharReceived(void* handle);

/**
 * @brief  Gets new received characters on com port
 * @param  None
 * @retval Returns the character
 */
uint8_t HW_UART_GetNewChar(void* handle);

void HW_UART_Rx_Callback(void* instance);

/* Exported macros -----------------------------------------------------------*/
//#if 1
//#define PRINTF(...)     HW_UART_Send(HW_UART1_GetHandle(), __VA_ARGS__)
//#else
//#define PRINTF(...)
//#endif

#ifdef DEBUG
#define PRINTD(...)     PRINTF(__VA_ARGS__)
#else
#define PRINTD(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __HW_UART_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
