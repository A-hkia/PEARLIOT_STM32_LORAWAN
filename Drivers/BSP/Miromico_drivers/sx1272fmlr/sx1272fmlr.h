/*******************************************************************************
 * @file    sx1272fmlr.h
 * @author  MCD Application Team
 * @brief   driver sx1272fmlr board
 ******************************************************************************
*/

#ifndef __SX1272FMLR_H__
#define __SX1272FMLR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define BOARD_WAKEUP_TIME  0 // no TCXO

/* Exported functions ------------------------------------------------------- */

/*!
 * \brief Initializes the radio I/Os pins interface
 */
void SX1272IoInit(void);

/*!
 * \brief De-initializes the radio I/Os pins interface.
 *
 * \remark Useful when going in MCU lowpower modes
 */
void SX1272IoDeInit(void);

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
bool SX1272CheckRfFrequency(uint32_t frequency);

/*!
 * Radio hardware and global parameters
 */
extern SX1272_t SX1272;

#ifdef __cplusplus
}
#endif

#endif /* __SX1272FMLR_H__*/
