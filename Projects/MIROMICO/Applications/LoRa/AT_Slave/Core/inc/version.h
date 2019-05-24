#ifndef __VERSION_H__
#define __VERSION_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lora_mac_version.h"
#include "lora.h"
#include "bsp_version.h"
/* Exported constants --------------------------------------------------------*/
#define VERSION   (uint32_t) ( LORA_MAC_VERSION | LRWAN_VERSION | LRWAN_MIRO_REVESION )
#define APP_VERSION_STRING   "1.0.1"

/* Exported types ------------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /*__VERSION_H__*/
