/*******************************************************************************
 * @file    hw_id.c
 * @author  Alex Raimondi
 * @version V1.0.2
 * @date    15-November-2016
 * @brief   System id functions
 ******************************************************************************
 */
#include "hw.h"
#include "hw_id.h"
#include "FMLR72_L0.h"

/*!
 *  \brief Unique Devices IDs register set ( STM32L1xxx )
 */
#define         ID1                                 ( 0x1FF80050 )
#define         ID2                                 ( 0x1FF80054 )
#define         ID3                                 ( 0x1FF80064 )

/**
  * @brief This function return a random seed
  * @note based on the device unique ID
  * @param None
  * @retval see
  */
uint32_t HW_GetRandomSeed(void)
{
  return ((*(uint32_t*)ID1) ^ (*(uint32_t*)ID2) ^ (*(uint32_t*)ID3));
}

/**
  * @brief This function return a unique ID
  * @param unique ID
  * @retval none
  */
__weak void HW_GetUniqueId(uint8_t* id)
{
  id[7] = ((*(uint32_t*)ID1) + (*(uint32_t*)ID3)) >> 24;
  id[6] = ((*(uint32_t*)ID1) + (*(uint32_t*)ID3)) >> 16;
  id[5] = ((*(uint32_t*)ID1) + (*(uint32_t*)ID3)) >> 8;
  id[4] = ((*(uint32_t*)ID1) + (*(uint32_t*)ID3));
  id[3] = ((*(uint32_t*)ID2)) >> 24;
  id[2] = ((*(uint32_t*)ID2)) >> 16;
  id[1] = ((*(uint32_t*)ID2)) >> 8;
  id[0] = ((*(uint32_t*)ID2));
}
