/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : ota.h
* @version        : v 1.0.0
* @brief          : ExpressLink OTA task
******************************************************************************
* @attention
*
* Copyright (c) 2022 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
******************************************************************************
*/

/* USER CODE END Header */

/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OTA_H__
#define __OTA_H__

#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes ------------------------------------------------------------------*/
#include "ExpressLink.h"

/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define OTA_BUFFER_SIZE  57    /* Specify how many HOTA bytes to read from  the module at a time. Keep in mind the binary data is sent as string */
#define DELAY_OTA_CLOSE  20000  /* Delay after sending the AT+OTA CLOSE command to end the OTA Job. */
#define MAX_HOTA_IMAGE_SIZE     2 * 1024 * 1024 // 4MB

#if (((OTA_BUFFER_SIZE * 2) + 14) > EXPRESSLINK_RX_BUFFER_SIZE)
#error "OTA_BUFFER_SIZE too big"
#endif

#define HOTA_START_ADDRESS  0

#define NO_OTA_IN_PROGRESS       0
#define NEW_MODULE_OTA_PROPOSED  1
#define NEW_HOST_OTA_PROPOSED    2
#define OTA_IN_PROGRESS          3
#define NEW_MODULE_IMAGE_ARRIVED 4
#define NEW_HOST_IMAGE_ARRIVED   5
  
/* Exported functions prototypes ---------------------------------------------*/

void vOTATask();

#ifdef __cplusplus
}
#endif

#endif /* __OTA_H__ */
