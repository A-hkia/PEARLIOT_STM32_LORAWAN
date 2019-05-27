/*
 ******************************************************************************
 * @file    hw_msp.h
 * @author  MCD Application Team
 * @version V1.0.2
 * @date    15-November-2016
 * @brief   Header for driver hw msp module
 ******************************************************************************
 */
#ifndef __HW_ID_H__
#define __HW_ID_H__

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t HW_GetRandomSeed(void);

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void HW_GetUniqueId(uint8_t* id);

#ifdef __cplusplus
}
#endif

#endif /* __HW_ID_H__ */
