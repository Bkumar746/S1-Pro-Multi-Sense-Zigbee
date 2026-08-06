#include "system_control.h"

#include "esp_check.h"
#include "esp_system.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

typedef enum {
    SYSTEM_CONTROL_RESTART,
    SYSTEM_CONTROL_FACTORY_RESET,
} system_control_request_t;

static QueueHandle_t s_request_queue;

static void system_control_task(void *argument)
{
    (void)argument;

    while (true) {
        system_control_request_t request;
        if (xQueueReceive(s_request_queue, &request, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

        if (request == SYSTEM_CONTROL_RESTART) {
            esp_restart();
        }

        const esp_err_t status = nvs_flash_erase();
        if (status != ESP_OK) {
            continue;
        }

        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_factory_reset();
        esp_zb_lock_release();

        esp_restart();
    }
}

static bool queue_request(system_control_request_t request)
{
    if (s_request_queue == NULL) {
        return false;
    }
    return xQueueSend(s_request_queue, &request, 0) == pdTRUE;
}

void system_control_start(void)
{
    s_request_queue = xQueueCreate(1U, sizeof(system_control_request_t));
    ESP_ERROR_CHECK(s_request_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    const BaseType_t created =
        xTaskCreate(system_control_task, "system_control", 3072, NULL, 4, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

bool system_control_request_restart(void)
{
    return queue_request(SYSTEM_CONTROL_RESTART);
}

bool system_control_request_factory_reset(void)
{
    return queue_request(SYSTEM_CONTROL_FACTORY_RESET);
}
