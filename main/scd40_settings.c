#include "scd40_settings.h"

#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "s1_pro_config.h"

static const char *NVS_NAMESPACE = "scd40_cfg";
static const char *NVS_REFERENCE_KEY = "frc_ppm";

static SemaphoreHandle_t s_mutex;
static uint16_t s_calibration_reference_ppm =
    S1_PRO_SCD40_DEFAULT_CALIBRATION_REFERENCE_PPM;
static bool s_dirty;
static int64_t s_changed_at_us;

static bool reference_is_valid(uint16_t reference_ppm)
{
    return reference_ppm >= S1_PRO_SCD40_MIN_CALIBRATION_REFERENCE_PPM &&
           reference_ppm <= S1_PRO_SCD40_MAX_CALIBRATION_REFERENCE_PPM;
}

static esp_err_t save_reference(uint16_t reference_ppm)
{
    nvs_handle_t handle = 0;
    esp_err_t status = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (status == ESP_OK) {
        status = nvs_set_u16(handle, NVS_REFERENCE_KEY, reference_ppm);
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
        (int64_t)S1_PRO_SCD40_SETTINGS_SAVE_DELAY_MS * 1000LL;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
        uint16_t reference_ppm = 0U;
        bool save_due = false;

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        if (s_dirty && esp_timer_get_time() - s_changed_at_us >= delay_us) {
            reference_ppm = s_calibration_reference_ppm;
            s_dirty = false;
            save_due = true;
        }
        xSemaphoreGive(s_mutex);

        if (!save_due) {
            continue;
        }

        const esp_err_t status = save_reference(reference_ppm);
        if (status != ESP_OK) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            s_dirty = true;
            s_changed_at_us = esp_timer_get_time();
            xSemaphoreGive(s_mutex);
        }
    }
}

void scd40_settings_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    nvs_handle_t handle = 0;
    const esp_err_t status = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (status == ESP_OK) {
        uint16_t restored_reference_ppm = 0U;
        const esp_err_t read_status =
            nvs_get_u16(handle, NVS_REFERENCE_KEY, &restored_reference_ppm);
        if (read_status == ESP_OK &&
            reference_is_valid(restored_reference_ppm)) {
            s_calibration_reference_ppm = restored_reference_ppm;
        }
        nvs_close(handle);
    }

    const BaseType_t created =
        xTaskCreate(persistence_task, "scd40_settings", 3072, NULL, 2, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

uint16_t scd40_settings_get_calibration_reference_ppm(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const uint16_t reference_ppm = s_calibration_reference_ppm;
    xSemaphoreGive(s_mutex);
    return reference_ppm;
}

bool scd40_settings_set_calibration_reference_ppm(uint16_t reference_ppm)
{
    if (!reference_is_valid(reference_ppm)) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_calibration_reference_ppm != reference_ppm) {
        s_calibration_reference_ppm = reference_ppm;
        s_dirty = true;
        s_changed_at_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_mutex);
    return true;
}
