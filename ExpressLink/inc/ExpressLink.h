/**
******************************************************************************
* @file           : ExpressLink.h
* @version        : v 1.0.0
* @brief          : This file implements AWS ExpressLink driver header
******************************************************************************
* @attention
*
* Copyright (c) 2019 STMicroelectronics.
* All rights reserved.
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the
*                             opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/

/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef INC_EXPRESSLINK_H_
#define INC_EXPRESSLINK_H_

// Add Expresslink specific defines here

#define EXPRESSLINK_TX_BUFFER_SIZE 128
#define EXPRESSLINK_RX_BUFFER_SIZE 128

/* Private typedef -----------------------------------------------------------*/

#define EXPRESSLINK_LOG_LEVEL_0      0
#define EXPRESSLINK_LOG_LEVEL_1      1
#define EXPRESSLINK_LOG_LEVEL_2      2

#define ExpressLink_BOOT_DELAY       3000

#define ExpressLink_COMMAND_TIMEOUT  3000

#define ExpressLink_MAX_TOPIC_SIZE   128

#define ExpressLink_TechSpec "v1.1.1"

// #include <driver/uart.h>
#include <esp_err.h>
#include <stdbool.h>

/* USER CODE BEGIN PTD */
typedef struct MQTTPublishInfo_t
{
  char * pTopic;
  char * pPayload;
  uint32_t payloadLength;
} MQTTPublishInfo_t;


typedef enum ExpressLink_event_id_t
{
  EXPRESSLINK_EVENT_NONE = (uint32_t)0,
  EXPRESSLINK_EVENT_MESSAGE,
  EXPRESSLINK_EVENT_STARTUP,
  EXPRESSLINK_EVENT_CONLOST,
  EXPRESSLINK_EVENT_OVERRUN,
  EXPRESSLINK_EVENT_OTA,
  EXPRESSLINK_EVENT_CONNECT,
  EXPRESSLINK_EVENT_CONFMODE,
  EXPRESSLINK_EVENT_SUBACK,
  EXPRESSLINK_EVENT_SUBNACK,
  EXPRESSLINK_EVENT_SHADOW_INIT = (uint32_t)20,
  EXPRESSLINK_EVENT_SHADOW_INIT_FAILED,
  EXPRESSLINK_EVENT_SHADOW_INIT_DOC,
  EXPRESSLINK_EVENT_SHADOW_UPDATE,
  EXPRESSLINK_EVENT_SHADOW_DELTA,
  EXPRESSLINK_EVENT_SHADOW_DELETE,
  EXPRESSLINK_EVENT_SHADOW_SUBACK,
  EXPRESSLINK_EVENT_SHADOW_SUBNACK,
  EXPRESSLINK_EVENT_MATTER_EVENT = 1005,
} ExpressLink_event_id_t;

typedef struct ExpressLink_event_t
{
  ExpressLink_event_id_t id;
  uint32_t param;
} ExpressLink_event_t;

typedef enum ExpressLink_state_t
{
  EXPRESSLINK_STATE_INIT      = (uint32_t)(0),
  EXPRESSLINK_STATE_READY     = (uint32_t)(1<<0),
  EXPRESSLINK_STATE_CONNECTED = (uint32_t)(1<<2),
  EXPRESSLINK_STATE_CONNECT   = (uint32_t)(1<<3),
  EXPRESSLINK_STATE_SLEEP     = (uint32_t)(1<<4),
  EXPRESSLINK_STATE_ERROR     = (uint32_t)(1<<5),
} ExpressLink_state_t;

typedef struct ExpressLink_OTA_Data_t
{
  uint32_t count;
  uint8_t* data;
  uint32_t crc;
} ExpressLink_OTA_Data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

esp_err_t ExpressLink_Interface_Init(void);

/* USER CODE END PD */
void   ExpressLink_Reset                 (void);
char * ExpressLink_Connect               (void);
char * ExpressLink_MatterStart           (void);
char * ExpressLink_ConnectAsync          (void);
char * ExpressLink_Disonnect             (void);
char * ExpressLink_Sleep                 (int duration);
char * ExpressLink_SendMessage           (int topicNumber, char * message);
void   ExpressLink_EventCallback         (ExpressLink_event_t event);
char * ExpressLink_SubscribeToTopic      (int topicNumber);
char * ExpressLink_UnsubscribeFromTopic  (int topicNumber);

int      ExpressLink_OTA_GetState        (void);
char *   ExpressLink_OTA_Accept          (void);
char *   ExpressLink_OTA_Apply           (void);
char *   ExpressLink_OTA_Close           (void);
char *   ExpressLink_OTA_Flush           (void);
uint32_t ExpressLink_OTA_Read            (ExpressLink_OTA_Data_t * OTA_Data, uint32_t byte_count);
char *   ExpressLink_OTA_Seek            (uint32_t address);

char * ExpressLink_SetEndpoint           (const char * endpoint);
char * ExpressLink_SetSSID               (const char * ssid);
char * ExpressLink_SetPassphrase         (const char * passphrase);
char * ExpressLink_SetAPN                (const char * apn);
char * ExpressLink_SetTopic              (int topicNumber, const char * topic);
char * ExpressLink_SetConfMode           (void);
char * ExpressLink_SetDefenderPeriod     (int period);
char * ExpressLink_SetQoS                (int QoS);
void   ExpressLink_SetState              (ExpressLink_state_t NewState);
char * ExpressLink_SetCustomName         (const char * CustomName);

char * ExpressLink_ShadowEnable          (void);
char * ExpressLink_ShadowDisable         (void);
char * ExpressLink_ShadowSetToken        (int token);
char * ExpressLink_ShadowInit            (int index);
char * ExpressLink_ShadowSubscribe       (int index);
char*  ExpressLink_ShadowUnSubscribe     (int index);
char * ExpressLink_ShadowSetTopic        (int topicNumber, char *topic);
char * ExpressLink_ShadowDoc             (int index);
char * ExpressLink_ShadowGetDoc          (int index);
char * ExpressLink_ShadowUpdate          (int index, char * new_state);
char * ExpressLink_ShadowGetUpdate       (int index);
char * ExpressLink_ShadowGetDelta        (int index);
char * ExpressLink_ShadowDelete          (int index);
char * ExpressLink_ShadowGetDelete       (int index);

ExpressLink_state_t ExpressLink_GetState (void);

bool ExpressLink_IsEvent(void);
ExpressLink_event_t ExpressLink_GetEvent (void);

///// Matter Reporting

char * ExpressLink_ReportTemp(uint8_t temp);

char *   ExpressLink_GetTime             (void);
char *   ExpressLink_GetConnection       (void);
char *   ExpressLink_GetThingName        (void);
char *   ExpressLink_GetConf             (char * conf);
void     ExpressLink_GetTopic            (char * topic);
uint32_t ExpressLink_GetMessage          (MQTTPublishInfo_t * msg);
uint32_t ExpressLink_GetMatterMessage    (MQTTPublishInfo_t * msg);
char*    ExpressLink_GetMessageTopic     (int index);

#define ExpressLink_GetSSID()        ExpressLink_GetConf((char *)&"SSID"       )
#define ExpressLink_GetAbout()       ExpressLink_GetConf((char *)&"About"      )
#define ExpressLink_GetVersion()     ExpressLink_GetConf((char *)&"Version"    )
#define ExpressLink_GetTechSpec()    ExpressLink_GetConf((char *)&"TechSpec"   )
#define ExpressLink_GetCertificate() ExpressLink_GetConf((char *)&"Certificate")
#define ExpressLink_GetAPN()         ExpressLink_GetConf((char *)&"APN"        )
#define ExpressLink_GetCustomName()  ExpressLink_GetConf((char *)&"CustomName" )

#endif /* INC_EXPRESSLINK_H_ */
