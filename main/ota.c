/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : ota.c
 * @version        : v 1.0.0
 * @brief          : ExpressLink OTA/HOTA
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

/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/* Includes ------------------------------------------------------------------*/

#include "ota.h"

/* USER CODE BEGIN Includes */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "ExpressLink.h"
#include "esp_err.h"
#include "esp_log.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#define TAG "ota"

/* Private functions ---------------------------------------------------------*/
static esp_err_t ota_flash_erase(void);
static esp_err_t ota_flash_write(const uint32_t address, uint8_t *data, const uint32_t size);
static void ota_do_hota(void);
static void ota_flash_swap_bank(void);

/* Private variables ---------------------------------------------------------*/
static uint8_t ota_data[OTA_BUFFER_SIZE];

/* User code ---------------------------------------------------------*/
void vOTATask()
{
    switch (ExpressLink_OTA_GetState())
    {
    /**********************************************/
    case NO_OTA_IN_PROGRESS:
      ESP_LOGI(TAG, "No OTA in progress");

      ExpressLink_OTA_Flush();
      break;

      /**********************************************/
    case NEW_MODULE_OTA_PROPOSED:
      ESP_LOGI(TAG, "A new module OTA update is being proposed");

      ExpressLink_OTA_Accept();
      break;

      /**********************************************/
    case NEW_HOST_OTA_PROPOSED:
      ESP_LOGI(TAG, "A new Host OTA update is being proposed");
      ExpressLink_OTA_Accept();
      break;

      /**********************************************/
    case OTA_IN_PROGRESS:
      ESP_LOGI(TAG, "OTA in progress.");
      break;

      /**********************************************/
    case NEW_MODULE_IMAGE_ARRIVED:
      ESP_LOGI(TAG, "A new module firmware image has arrived.");

      ExpressLink_OTA_Close();
      ExpressLink_OTA_Apply();
      vTaskDelay(pdMS_TO_TICKS(DELAY_OTA_CLOSE));

      break;

      /**********************************************/
    case NEW_HOST_IMAGE_ARRIVED:
      ESP_LOGI(TAG, "A new Host image has arrived.");

      //taskENTER_CRITICAL(); /* We need to suspend interrupts to not disturb USART communication and risk to corrupt received messages */
      ota_do_hota();
      //taskEXIT_CRITICAL();
      break;

      /**********************************************/
    default:
      ExpressLink_OTA_Flush();
      break;
    }
}

static void ota_do_hota(void)
{
  int ota_state = 0;
  uint32_t address = HOTA_START_ADDRESS; //FLASH_START_ADDRESS + FLASH_BANK_SIZE;
  ExpressLink_OTA_Data_t OTA_Data;
  uint32_t hota_image_size = 0;

  OTA_Data.data = ota_data;

  if (ota_flash_erase() != ESP_OK)
  {
	ESP_LOGE(TAG, "Flash Erase error");
    ExpressLink_OTA_Flush();
    return;
  }

  do
  {
    if (ExpressLink_OTA_Read(&OTA_Data, OTA_BUFFER_SIZE) > 0) {
        hota_image_size += OTA_Data.count;

        if (hota_image_size <= MAX_HOTA_IMAGE_SIZE)
        {
            ota_flash_write(address, (uint8_t*) ota_data, OTA_Data.count);
        }

        address += OTA_Data.count;
    }
    ESP_LOGI(TAG, "\r[INFO] hota_image_size %d", (int ) hota_image_size);
  }
  while (OTA_Data.count == OTA_BUFFER_SIZE);

  if (hota_image_size <= MAX_HOTA_IMAGE_SIZE)
  {
    ExpressLink_OTA_Close();

    ESP_LOGI(TAG, "\n");

    do
    {
      ota_state = ExpressLink_OTA_GetState();
      ESP_LOGI(TAG, ".");
      ESP_LOGD(TAG, "OTA State %d", ota_state);
    }
    while (ota_state != 0);

    ESP_LOGI(TAG, "");

    //taskEXIT_CRITICAL();

    ESP_LOGI(TAG, "Rebooting\n");

    vTaskDelay(pdMS_TO_TICKS(2000));
    ota_flash_swap_bank();
  }
  else
  {
    ESP_LOGE(TAG, "HOTA Image 0x%08X exceeds MAX_HOTA_IMAGE_SIZE 0x%08X ", (int ) hota_image_size, (int) MAX_HOTA_IMAGE_SIZE);

    ExpressLink_OTA_Flush();
  }

}

static esp_err_t ota_flash_write(const uint32_t address, uint8_t *data, const uint32_t size)
{
  ESP_LOGI(TAG, "Writing %" PRIu32 "bytes @0x%" PRIx32 "08X", size, address);
  return ESP_OK;
}

static void ota_flash_swap_bank(void)
{
  return;
}

static esp_err_t ota_flash_erase(void)
{
  return ESP_OK;
}
