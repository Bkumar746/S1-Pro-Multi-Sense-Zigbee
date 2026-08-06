#include "ltr390_sensor.h"

#include <stddef.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "ltr390_protocol.h"
#include "s1_pro_config.h"

static const char *TAG = "ltr390";

#define LTR390_REG_MAIN_CTRL 0x00U
#define LTR390_REG_MEAS_RATE 0x04U
#define LTR390_REG_GAIN 0x05U
#define LTR390_REG_PART_ID 0x06U
#define LTR390_REG_MAIN_STATUS 0x07U
#define LTR390_REG_ALS_DATA 0x0DU
#define LTR390_REG_UVS_DATA 0x10U

#define LTR390_CTRL_STANDBY 0x00U
#define LTR390_CTRL_ALS_ACTIVE 0x02U
#define LTR390_CTRL_UVS_ACTIVE 0x0AU
#define LTR390_STATUS_NEW_DATA 0x08U
#define LTR390_EXPECTED_PART_NUMBER 0xB0U

#define LTR390_ALS_MEAS_RATE 0x22U
#define LTR390_ALS_GAIN 0x01U
#define LTR390_ALS_READY_TIMEOUT_MS 250U
#define LTR390_UVS_MEAS_RATE 0x04U
#define LTR390_UVS_GAIN 0x04U
#define LTR390_UVS_READY_TIMEOUT_MS 700U
#define LTR390_STATUS_POLL_MS 20U
#define LTR390_MAX_CONSECUTIVE_ERRORS 3U

static portMUX_TYPE s_snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static ltr390_snapshot_t s_snapshot;
static i2c_master_dev_handle_t s_device;

static void record_error(void)
{
    portENTER_CRITICAL(&s_snapshot_lock);
    ++s_snapshot.errors;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

static esp_err_t write_register(uint8_t register_address, uint8_t value)
{
    const uint8_t transaction[] = {register_address, value};
    return i2c_master_transmit(s_device, transaction, sizeof(transaction),
                               S1_PRO_I2C_TIMEOUT_MS);
}

static esp_err_t read_registers(uint8_t register_address, uint8_t *data,
                                size_t data_size)
{
    ESP_RETURN_ON_FALSE(data != NULL && data_size > 0U, ESP_ERR_INVALID_ARG,
                        TAG, "Invalid register read buffer");
    return i2c_master_transmit_receive(s_device, &register_address, 1U, data,
                                       data_size, S1_PRO_I2C_TIMEOUT_MS);
}

static esp_err_t read_register(uint8_t register_address, uint8_t *value)
{
    return read_registers(register_address, value, 1U);
}

static esp_err_t configure_mode(uint8_t measurement_rate, uint8_t gain,
                                uint8_t control)
{
    ESP_RETURN_ON_ERROR(write_register(LTR390_REG_MAIN_CTRL,
                                       LTR390_CTRL_STANDBY),
                        TAG, "Unable to stop LTR390 conversion");
    ESP_RETURN_ON_ERROR(write_register(LTR390_REG_MEAS_RATE, measurement_rate),
                        TAG, "Unable to set LTR390 measurement rate");
    ESP_RETURN_ON_ERROR(write_register(LTR390_REG_GAIN, gain), TAG,
                        "Unable to set LTR390 gain");
    uint8_t ignored_status = 0;
    ESP_RETURN_ON_ERROR(read_register(LTR390_REG_MAIN_STATUS, &ignored_status),
                        TAG, "Unable to clear LTR390 data-ready status");
    ESP_RETURN_ON_ERROR(write_register(LTR390_REG_MAIN_CTRL, control), TAG,
                        "Unable to start LTR390 conversion");
    return ESP_OK;
}

static esp_err_t wait_for_new_data(uint32_t timeout_ms)
{
    uint32_t elapsed_ms = 0;
    while (elapsed_ms < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(LTR390_STATUS_POLL_MS));
        elapsed_ms += LTR390_STATUS_POLL_MS;
        uint8_t status = 0;
        ESP_RETURN_ON_ERROR(read_register(LTR390_REG_MAIN_STATUS, &status), TAG,
                            "LTR390 status read failed");
        if ((status & LTR390_STATUS_NEW_DATA) != 0U) {
            return ESP_OK;
        }
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t read_channel(uint8_t data_register, uint32_t *counts)
{
    uint8_t data[LTR390_DATA_SIZE];
    ESP_RETURN_ON_ERROR(read_registers(data_register, data, sizeof(data)), TAG,
                        "LTR390 channel read failed");
    *counts = ltr390_decode_20bit(data);
    return ESP_OK;
}

static esp_err_t measure_pair(uint32_t *ambient_counts, uint32_t *uv_counts)
{
    ESP_RETURN_ON_ERROR(
        configure_mode(LTR390_ALS_MEAS_RATE, LTR390_ALS_GAIN,
                       LTR390_CTRL_ALS_ACTIVE),
        TAG, "LTR390 ALS setup failed");
    ESP_RETURN_ON_ERROR(wait_for_new_data(LTR390_ALS_READY_TIMEOUT_MS), TAG,
                        "LTR390 ALS conversion timed out");
    ESP_RETURN_ON_ERROR(read_channel(LTR390_REG_ALS_DATA, ambient_counts), TAG,
                        "LTR390 ALS data read failed");

    ESP_RETURN_ON_ERROR(
        configure_mode(LTR390_UVS_MEAS_RATE, LTR390_UVS_GAIN,
                       LTR390_CTRL_UVS_ACTIVE),
        TAG, "LTR390 UVS setup failed");
    ESP_RETURN_ON_ERROR(wait_for_new_data(LTR390_UVS_READY_TIMEOUT_MS), TAG,
                        "LTR390 UVS conversion timed out");
    ESP_RETURN_ON_ERROR(read_channel(LTR390_REG_UVS_DATA, uv_counts), TAG,
                        "LTR390 UVS data read failed");
    return ESP_OK;
}

static esp_err_t initialize_sensor(void)
{
    uint8_t part_id = 0;
    ESP_RETURN_ON_ERROR(read_register(LTR390_REG_PART_ID, &part_id), TAG,
                        "LTR390 identity read failed");
    ESP_RETURN_ON_FALSE((part_id & 0xF0U) == LTR390_EXPECTED_PART_NUMBER,
                        ESP_ERR_INVALID_RESPONSE, TAG,
                        "Unexpected LTR390 part ID 0x%02x", part_id);

    portENTER_CRITICAL(&s_snapshot_lock);
    s_snapshot.part_id = part_id;
    portEXIT_CRITICAL(&s_snapshot_lock);
    return ESP_OK;
}

static void store_measurement(uint32_t ambient_counts, uint32_t uv_counts)
{
    portENTER_CRITICAL(&s_snapshot_lock);
    s_snapshot.received_at_us = esp_timer_get_time();
    s_snapshot.ambient_light_centilux =
        ltr390_als_counts_to_centilux(ambient_counts);
    s_snapshot.uv_index_centi = ltr390_uv_counts_to_centi_index(uv_counts);
    s_snapshot.valid = true;
    ++s_snapshot.measurement_count;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

static bool detect_sensor(void)
{
    const esp_err_t status = i2c_master_probe(
        s1_pro_i2c_bus_get(), S1_PRO_LTR390_I2C_ADDRESS,
        S1_PRO_I2C_TIMEOUT_MS);
    if (status == ESP_OK) {
        return true;
    }
    return false;
}

static void ltr390_task(void *argument)
{
    (void)argument;
    bool device_added = false;

    while (true) {
        if (!detect_sensor()) {
            vTaskDelay(pdMS_TO_TICKS(S1_PRO_LTR390_DETECT_RETRY_MS));
            continue;
        }
        if (!device_added) {
            const esp_err_t status = s1_pro_i2c_add_device(
                S1_PRO_LTR390_I2C_ADDRESS, &s_device);
            if (status != ESP_OK) {
                record_error();
                vTaskDelay(pdMS_TO_TICKS(S1_PRO_LTR390_DETECT_RETRY_MS));
                continue;
            }
            device_added = true;
        }
        if (initialize_sensor() != ESP_OK) {
            record_error();
            vTaskDelay(pdMS_TO_TICKS(S1_PRO_LTR390_DETECT_RETRY_MS));
            continue;
        }

        uint8_t consecutive_errors = 0;
        TickType_t last_wake = xTaskGetTickCount();
        while (consecutive_errors < LTR390_MAX_CONSECUTIVE_ERRORS) {
            uint32_t ambient_counts = 0;
            uint32_t uv_counts = 0;
            const esp_err_t status = measure_pair(&ambient_counts, &uv_counts);
            if (status == ESP_OK) {
                consecutive_errors = 0;
                store_measurement(ambient_counts, uv_counts);
            } else {
                ++consecutive_errors;
                record_error();
            }
            vTaskDelayUntil(&last_wake,
                            pdMS_TO_TICKS(S1_PRO_LTR390_MEASUREMENT_PERIOD_MS));
        }
    }
}

void ltr390_sensor_get_snapshot(ltr390_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_snapshot_lock);
    *snapshot = s_snapshot;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

void ltr390_sensor_start(void)
{
    const BaseType_t created =
        xTaskCreate(ltr390_task, "ltr390", 4096, NULL, 4, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
