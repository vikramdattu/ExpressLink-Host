/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>

// #include "esp_flash.h"
#include "esp_log.h"
// #include "esp_cpu.h"
#include "inttypes.h"

#include "ExpressLink.h"

#define TAG "app"

#define EXL_DEMO_TASK_Q_SIZE 20
#define EXL_DEMO_TASK_STACK_SIZE (10 * 1024)
#define EXL_DEMO_TASK_PRIORITY 4

const char *AWS_IOT_CORE_ENDPOINT = "<hash>-ats.iot.<region>.amazonaws.com";

static MQTTPublishInfo_t mqtt_pub_info;
static char payloadBuf[EXPRESSLINK_RX_BUFFER_SIZE];
static char topic[EXPRESSLINK_RX_BUFFER_SIZE];

typedef void (*exl_demo_work_fn_t)(void *priv_data);

typedef struct {
    exl_demo_work_fn_t work_fn;
    void *priv_data;
} exl_demo_work_queue_entry_t;

static QueueHandle_t work_queue;

static void exl_demo_handle_work_queue(void)
{
    exl_demo_work_queue_entry_t work_queue_entry;
    /* 2 sec delay to prevent spinning */
    BaseType_t ret = xQueueReceive(work_queue, &work_queue_entry, 2000 / portTICK_PERIOD_MS);
    while (ret == pdTRUE) {
        work_queue_entry.work_fn(work_queue_entry.priv_data);
        ret = xQueueReceive(work_queue, &work_queue_entry, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void exl_demo_work_queue_task(void *param)
{
    ESP_LOGI(TAG, "EXL Demo Work Queue task started.");
    while (1) {
        exl_demo_handle_work_queue();
    }
    ESP_LOGI(TAG, "Stopping Work Queue task");
    vTaskDelete(NULL);
}

esp_err_t exl_demo_work_queue_add_task(exl_demo_work_fn_t work_fn, void *priv_data)
{
    if (!work_queue) {
        ESP_LOGE(TAG, "Cannot enqueue function as Work Queue hasn't been created.");
        return ESP_ERR_INVALID_STATE;
    }
    exl_demo_work_queue_entry_t work_queue_entry = {
        .work_fn = work_fn,
        .priv_data = priv_data,
    };
    if (xQueueSend(work_queue, &work_queue_entry, 0) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t exl_demo_work_queue_start(void)
{
    work_queue = xQueueCreate(EXL_DEMO_TASK_Q_SIZE, sizeof(exl_demo_work_queue_entry_t));
    if (!work_queue) {
        ESP_LOGE(TAG, "Failed to create Work Queue.");
        return ESP_FAIL;
    }

    if (xTaskCreate(&exl_demo_work_queue_task, "exl_queue_task", EXL_DEMO_TASK_STACK_SIZE,
                    NULL, EXL_DEMO_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Couldn't create work queue task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t expresslink_parse_execute_matter_payload(const char *data, size_t data_len)
{
    ESP_LOGI(TAG, "Received matter_message %.*s", data_len, data);
    ESP_LOGW(TAG, "%s not implemented/tested yet", __func__);
    return ESP_ERR_NOT_SUPPORTED;
}

void expresslink_get_and_process_evt(void *data)
{
    ExpressLink_event_t event = ExpressLink_GetEvent();
    if (event.id != EXPRESSLINK_EVENT_NONE) {
        ESP_LOGI(TAG, "Expresslink Event %d", event.id);
    }

    switch (event.id) {
        case EXPRESSLINK_EVENT_NONE:
            // ESP_LOGI(TAG, "No event found");
            break;
        case EXPRESSLINK_EVENT_MATTER_EVENT: {
                ESP_LOGI(TAG, "Getting the message...");
                uint32_t msg_len = ExpressLink_GetMatterMessage(&mqtt_pub_info);
                expresslink_parse_execute_matter_payload(mqtt_pub_info.pPayload, msg_len);
            }
            break;
        case EXPRESSLINK_EVENT_MESSAGE: {
                uint32_t msg_len = ExpressLink_GetMessage(&mqtt_pub_info);
                if (msg_len > 0) {
                    ESP_LOGI(TAG, "message is topic %s, payload %s len %"PRIu32,
                                mqtt_pub_info.pTopic, mqtt_pub_info.pPayload, msg_len);
                }
            }
            break;
            case EXPRESSLINK_EVENT_STARTUP:
                ExpressLink_SetEndpoint(AWS_IOT_CORE_ENDPOINT);
                vTaskDelay(1);

                ExpressLink_Connect();
                vTaskDelay(1);
                ExpressLink_SetTopic(1, "MyTopic1");
                ExpressLink_SetTopic(2, "MyTopic2");
                vTaskDelay(1);
                ExpressLink_SubscribeToTopic(1);

                char *ret_val = ExpressLink_MatterStart();
                printf("Matter Start response: %s", ret_val);
            break;
        default:
            // ESP_LOGW(TAG, "Unhandled event %d", event.id);
    }
}

void app_main(void)
{
    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    ESP_LOGI(TAG, "Setting up ExpressLink device");

    exl_demo_work_queue_start(); // initializes work task

    mqtt_pub_info.pPayload = payloadBuf;
    mqtt_pub_info.pTopic = topic;

    ExpressLink_Interface_Init();
    ExpressLink_Reset();

    ESP_LOGI(TAG, "Going into Expresslink event check loop...");

    while (1) {
        bool is_event = ExpressLink_IsEvent();
        if (is_event) {
            exl_demo_work_queue_add_task(&expresslink_get_and_process_evt, NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
