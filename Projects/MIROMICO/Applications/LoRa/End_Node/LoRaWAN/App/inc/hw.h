/******************************************************************************
 * @file    hw.h
 * @author  MCD Application Team
 * @version V1.0.2
 * @date    15-November-2016
 * @brief   contains all hardware driver
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_H__
#define __HW_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "hw_conf.h"
#include "debug.h"

/* BACKUP_PRIMASK MUST be implemented at the beginning of the function
   that implement a critical section
   PRIMASK is saved on STACK and recovered at the end of the function
   That way RESTORE_PRIMASK ensures that no irq would be triggered in case of
   unbalanced enable/disable, reentrant code etc...*/
#define BACKUP_PRIMASK()  uint32_t primask_bit= __get_PRIMASK()
#define DISABLE_IRQ() __disable_irq()
#define ENABLE_IRQ() __enable_irq()
#define RESTORE_PRIMASK() __set_PRIMASK(primask_bit)

/* preprocessor directive to align buffer*/
#define ALIGN(n)             __attribute__((aligned(n)))
#define NORETURN             __attribute__ ((noreturn))

/** Define for new line character(s) */
#define NL "\n\r"

typedef enum {
  HW_UNLOCKED = 0x00U,
  HW_LOCKED   = 0x01U
} HW_LockTypeDef;

#define HW_LOCK(__HANDLE__)               \
  do {                                    \
    if ((__HANDLE__)->Lock == HW_LOCKED)  \
    {                                     \
      return;                             \
    }                                     \
    else                                  \
    {                                     \
      (__HANDLE__)->Lock = HW_LOCKED;     \
    }                                     \
  } while (0)

#define HW_UNLOCK(__HANDLE__)             \
  do {                                    \
    (__HANDLE__)->Lock = HW_UNLOCKED;     \
  } while (0)

/** MCU reset source definition */
typedef enum {
  eResetNone,
  eResetWU,
  eResetPIN,
  eResetLPW,
  eResetSW,
  eResetPOR,
  eResetIWDG,
  eResetWWDG
} eResetSrc_t;

/*!
 * \brief Initializes the boards peripherals.
 */
eResetSrc_t HW_Init();

/*!
* \brief De-initializes the target board peripherals to decrease power
*        consumption.
*/
void HW_DeInit(void);

/**
  * @brief This function Initializes the hardware Ios
  * @param None
  * @retval None
  */
void HW_IoInit(void);

/**
  * @brief This function Deinitializes the hardware Ios
  * @param None
  * @retval None
  */
void HW_IoDeInit(void);

/*!
 * \brief Configures the sytem Clock at start-up
 *
 * \param none
 * \retval none
 */
void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* __HW_H__ */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
