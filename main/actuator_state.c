#include "actuator_state.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "s1_pro_config.h"

static const char *NVS_NAMESPACE = "outputs";
static const char *NVS_KEY = "state";

#define OUTPUT_STATE_VERSION 1U

typedef struct {
    uint8_t version;
    uint8_t led_power;
    uint8_t led_level;
    uint8_t buzzer_power;
    uint16_t led_x;
    uint16_t led_y;
} stored_output_state_t;

static actuator_state_snapshot_t s_state = {
    .led_power = false,
    .led_level = S1_PRO_RGB_LED_DEFAULT_LEVEL,
    .led_x = S1_PRO_RGB_LED_DEFAULT_X,
    .led_y = S1_PRO_RGB_LED_DEFAULT_Y,
    .buzzer_power = false,
};
static SemaphoreHandle_t s_mutex;
static bool s_dirty;
static int64_t s_changed_at_us;

static bool stored_state_is_valid(const stored_output_state_t *stored)
{
    return stored->version == OUTPUT_STATE_VERSION &&
           stored->led_power <= 1U && stored->buzzer_power <= 1U &&
           stored->led_level <= 254U && stored->led_y != 0U;
}

static void mark_changed(void)
{
    s_dirty = true;
    s_changed_at_us = esp_timer_get_time();
}

static esp_err_t save_state(const actuator_state_snapshot_t *state)
{
    const stored_output_state_t stored = {
        .version = OUTPUT_STATE_VERSION,
        .led_power = state->led_power,
        .led_level = state->led_level,
        .buzzer_power = state->buzzer_power,
        .led_x = state->led_x,
        .led_y = state->led_y,
    };
    nvs_handle_t handle = 0;
    esp_err_t status = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (status == ESP_OK) {
        status = nvs_set_blob(handle, NVS_KEY, &stored, sizeof(stored));
    }
    if (status == ESP_OK) {
        status = nvs_commit(handle);
    }
    if (handle != 0) {
        nvs_close(handle);
    }
    return status;
}

static void persistence_task(void *argument)
{
    (void)argument;
    const int64_t delay_us =
        (int64_t)S1_PRO_OUTPUT_STATE_SAVE_DELAY_MS * 1000LL;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
        actuator_state_snapshot_t snapshot;
        bool save_due = false;

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        if (s_dirty && esp_timer_get_time() - s_changed_at_us >= delay_us) {
            snapshot = s_state;
            s_dirty = false;
            save_due = true;
        }
        xSemaphoreGive(s_mutex);

        if (!save_due) {
            continue;
        }

        const esp_err_t status = save_state(&snapshot);
        if (status != ESP_OK) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            s_dirty = true;
            s_changed_at_us = esp_timer_get_time();
            xSemaphoreGive(s_mutex);
        }
    }
}

void actuator_state_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    nvs_handle_t handle = 0;
    esp_err_t status = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (status == ESP_OK) {
        stored_output_state_t stored = {0};
        size_t size = sizeof(stored);
        status = nvs_get_blob(handle, NVS_KEY, &stored, &size);
        if (status == ESP_OK && size == sizeof(stored) &&
            stored_state_is_valid(&stored)) {
            s_state.led_power = stored.led_power != 0U;
            s_state.led_level = stored.led_level;
            s_state.led_x = stored.led_x;
            s_state.led_y = stored.led_y;
            s_state.buzzer_power = stored.buzzer_power != 0U;
        }
        nvs_close(handle);
    }

    BaseType_t created =
        xTaskCreate(persistence_task, "output_state", 3072, NULL, 2, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

void actuator_state_get(actuator_state_snapshot_t *state)
{
    if (state == NULL) {
        return;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *state = s_state;
    xSemaphoreGive(s_mutex);
}

void actuator_state_set_led_power(bool power)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_state.led_power != power) {
        s_state.led_power = power;
        mark_changed();
    }
    xSemaphoreGive(s_mutex);
}

void actuator_state_set_led_level(uint8_t level)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_state.led_level != level) {
        s_state.led_level = level;
        mark_changed();
    }
    xSemaphoreGive(s_mutex);
}

void actuator_state_set_led_color(uint16_t x, uint16_t y)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_state.led_x != x || s_state.led_y != y) {
        s_state.led_x = x;
        s_state.led_y = y;
        mark_changed();
    }
    xSemaphoreGive(s_mutex);
}

void actuator_state_set_buzzer_power(bool power)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_state.buzzer_power != power) {
        s_state.buzzer_power = power;
        mark_changed();
    }
    xSemaphoreGive(s_mutex);
}
