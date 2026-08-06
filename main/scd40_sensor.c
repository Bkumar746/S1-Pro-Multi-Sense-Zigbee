#include "scd40_sensor.h"

#include <stddef.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "nvs.h"
#include "s1_pro_config.h"
#include "scd4x_protocol.h"

static const char *TAG = "scd40";

#define SCD4X_CMD_START_PERIODIC_MEASUREMENT 0x21B1U
#define SCD4X_CMD_READ_MEASUREMENT 0xEC05U
#define SCD4X_CMD_STOP_PERIODIC_MEASUREMENT 0x3F86U
#define SCD4X_CMD_GET_DATA_READY_STATUS 0xE4B8U
#define SCD4X_CMD_GET_SERIAL_NUMBER 0x3682U
#define SCD4X_CMD_PERFORM_FORCED_RECALIBRATION 0x362FU
#define SCD4X_CMD_PERFORM_FACTORY_RESET 0x3632U
#define SCD4X_CMD_SET_TEMPERATURE_OFFSET 0x241DU
#define SCD4X_CMD_GET_TEMPERATURE_OFFSET 0x2318U
#define SCD4X_CMD_PERSIST_SETTINGS 0x3615U
#define SCD4X_WORD_RESPONSE_SIZE 3U
#define SCD4X_SERIAL_RESPONSE_SIZE 9U
#define SCD4X_MAX_CONSECUTIVE_ERRORS 3U
#define SCD4X_STOP_DELAY_MS 500U
#define SCD4X_FORCED_RECALIBRATION_DELAY_MS 400U
#define SCD4X_FACTORY_RESET_DELAY_MS 1200U
#define SCD4X_SETTING_DELAY_MS 1U
#define SCD4X_PERSIST_SETTINGS_DELAY_MS 800U
#define SCD4X_FORCED_RECALIBRATION_MIN_RUNTIME_MS (3U * 60U * 1000U)
#define SCD40_SETTINGS_NVS_NAMESPACE "scd40_cfg"
#define SCD40_OFFSET_SERIAL_NVS_KEY "offset_sn"

typedef enum {
    SCD40_CONTROL_FORCED_CALIBRATION,
    SCD40_CONTROL_FACTORY_RESET,
    SCD40_CONTROL_SET_TEMPERATURE_OFFSET,
} scd40_control_type_t;

typedef struct {
    scd40_control_type_t type;
    uint16_t reference_ppm;
    uint16_t temperature_offset_centi_c;
} scd40_control_request_t;

static portMUX_TYPE s_snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static scd40_snapshot_t s_snapshot;
static i2c_master_dev_handle_t s_device;
static bool s_operational;
static int64_t s_periodic_started_at_us;
static QueueHandle_t s_control_queue;

static esp_err_t set_and_persist_temperature_offset(
    uint16_t requested_offset_centi_c,
    uint16_t *confirmed_offset_centi_c);

static void record_error(void)
{
    portENTER_CRITICAL(&s_snapshot_lock);
    ++s_snapshot.errors;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

static esp_err_t send_command(uint16_t command)
{
    const uint8_t data[] = {
        (uint8_t)(command >> 8U),
        (uint8_t)(command & 0xFFU),
    };
    return i2c_master_transmit(s_device, data, sizeof(data),
                               S1_PRO_I2C_TIMEOUT_MS);
}

static esp_err_t send_command_with_word(uint16_t command, uint16_t value)
{
    uint8_t data[2U + SCD4X_WORD_FRAME_SIZE] = {
        (uint8_t)(command >> 8U),
        (uint8_t)(command & 0xFFU),
    };
    scd4x_encode_word(value, &data[2]);
    return i2c_master_transmit(s_device, data, sizeof(data),
                               S1_PRO_I2C_TIMEOUT_MS);
}

static esp_err_t read_command(uint16_t command, uint8_t *response,
                              size_t response_size)
{
    ESP_RETURN_ON_FALSE(response != NULL && response_size > 0U,
                        ESP_ERR_INVALID_ARG, TAG, "Invalid read buffer");
    ESP_RETURN_ON_ERROR(send_command(command), TAG,
                        "Command 0x%04x write failed", command);
    vTaskDelay(pdMS_TO_TICKS(1));
    return i2c_master_receive(s_device, response, response_size,
                              S1_PRO_I2C_TIMEOUT_MS);
}

static bool validate_words(const uint8_t *response, size_t response_size)
{
    if (response == NULL || response_size % 3U != 0U) {
        return false;
    }
    for (size_t offset = 0; offset < response_size; offset += 3U) {
        if (scd4x_crc8(&response[offset], 2U) != response[offset + 2U]) {
            return false;
        }
    }
    return true;
}

static uint16_t read_be16(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8U) | data[1];
}

static esp_err_t read_serial_number(uint64_t *serial_number)
{
    uint8_t response[SCD4X_SERIAL_RESPONSE_SIZE];
    ESP_RETURN_ON_ERROR(
        read_command(SCD4X_CMD_GET_SERIAL_NUMBER, response, sizeof(response)),
        TAG, "Serial-number read failed");
    ESP_RETURN_ON_FALSE(validate_words(response, sizeof(response)),
                        ESP_ERR_INVALID_CRC, TAG,
                        "Serial-number CRC validation failed");
    *serial_number = ((uint64_t)read_be16(&response[0]) << 32U) |
                     ((uint64_t)read_be16(&response[3]) << 16U) |
                     read_be16(&response[6]);
    return ESP_OK;
}

static esp_err_t get_data_ready(bool *ready)
{
    uint8_t response[SCD4X_WORD_RESPONSE_SIZE];
    ESP_RETURN_ON_ERROR(
        read_command(SCD4X_CMD_GET_DATA_READY_STATUS, response,
                     sizeof(response)),
        TAG, "Data-ready read failed");
    ESP_RETURN_ON_FALSE(validate_words(response, sizeof(response)),
                        ESP_ERR_INVALID_CRC, TAG,
                        "Data-ready CRC validation failed");
    *ready = (read_be16(response) & 0x07FFU) != 0U;
    return ESP_OK;
}

static esp_err_t read_measurement(scd4x_measurement_t *measurement)
{
    uint8_t response[SCD4X_MEASUREMENT_RESPONSE_SIZE];
    ESP_RETURN_ON_ERROR(
        read_command(SCD4X_CMD_READ_MEASUREMENT, response, sizeof(response)),
        TAG, "Measurement read failed");
    ESP_RETURN_ON_FALSE(scd4x_decode_measurement(response, measurement),
                        ESP_ERR_INVALID_CRC, TAG,
                        "Measurement CRC validation failed");
    return ESP_OK;
}

static esp_err_t read_temperature_offset(uint16_t *offset_centi_c)
{
    ESP_RETURN_ON_FALSE(offset_centi_c != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Missing temperature-offset output buffer");
    uint8_t response[SCD4X_WORD_RESPONSE_SIZE];
    ESP_RETURN_ON_ERROR(
        read_command(SCD4X_CMD_GET_TEMPERATURE_OFFSET, response,
                     sizeof(response)),
        TAG, "SCD40 temperature-offset read failed");
    uint16_t raw_offset = 0U;
    ESP_RETURN_ON_FALSE(scd4x_decode_word(response, &raw_offset),
                        ESP_ERR_INVALID_CRC, TAG,
                        "SCD40 temperature-offset CRC validation failed");
    *offset_centi_c =
        scd4x_temperature_offset_raw_to_centi_c(raw_offset);
    return ESP_OK;
}

static void store_temperature_offset(uint16_t offset_centi_c)
{
    portENTER_CRITICAL(&s_snapshot_lock);
    s_snapshot.temperature_offset_centi_c = offset_centi_c;
    s_snapshot.temperature_offset_valid = true;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

static bool product_temperature_offset_applied(uint64_t serial_number)
{
    nvs_handle_t handle = 0;
    const esp_err_t status = nvs_open(SCD40_SETTINGS_NVS_NAMESPACE,
                                      NVS_READONLY, &handle);
    if (status != ESP_OK) {
        return false;
    }

    uint64_t provisioned_serial_number = 0U;
    const esp_err_t read_status = nvs_get_u64(
        handle, SCD40_OFFSET_SERIAL_NVS_KEY, &provisioned_serial_number);
    nvs_close(handle);
    return read_status == ESP_OK &&
           provisioned_serial_number == serial_number;
}

static esp_err_t remember_product_temperature_offset(uint64_t serial_number)
{
    nvs_handle_t handle = 0;
    esp_err_t status = nvs_open(SCD40_SETTINGS_NVS_NAMESPACE,
                                NVS_READWRITE, &handle);
    if (status == ESP_OK) {
        status = nvs_set_u64(handle, SCD40_OFFSET_SERIAL_NVS_KEY,
                             serial_number);
    }
    if (status == ESP_OK) {
        status = nvs_commit(handle);
    }
    if (handle != 0) {
        nvs_close(handle);
    }
    return status;
}

static esp_err_t initialize_sensor(void)
{
    (void)send_command(SCD4X_CMD_STOP_PERIODIC_MEASUREMENT);
    vTaskDelay(pdMS_TO_TICKS(SCD4X_STOP_DELAY_MS));

    uint64_t serial_number = 0;
    ESP_RETURN_ON_ERROR(read_serial_number(&serial_number), TAG,
                        "SCD40 identity check failed");
    uint16_t temperature_offset_centi_c = 0U;
    ESP_RETURN_ON_ERROR(read_temperature_offset(&temperature_offset_centi_c),
                        TAG, "SCD40 temperature-offset initialization failed");
    if (!product_temperature_offset_applied(serial_number)) {
        ESP_RETURN_ON_ERROR(
            set_and_persist_temperature_offset(
                S1_PRO_SCD40_DEFAULT_TEMPERATURE_OFFSET_CENTI_C,
                &temperature_offset_centi_c),
            TAG, "Unable to provision product-default SCD40 Temp Offset");
        ESP_RETURN_ON_ERROR(
            remember_product_temperature_offset(serial_number), TAG,
            "Unable to remember provisioned SCD40 module serial number");
    }
    ESP_RETURN_ON_ERROR(send_command(SCD4X_CMD_START_PERIODIC_MEASUREMENT), TAG,
                        "Unable to start periodic measurement");

    portENTER_CRITICAL(&s_snapshot_lock);
    s_snapshot.serial_number = serial_number;
    s_snapshot.temperature_offset_centi_c = temperature_offset_centi_c;
    s_snapshot.temperature_offset_valid = true;
    portEXIT_CRITICAL(&s_snapshot_lock);
    return ESP_OK;
}

static void set_operational(bool operational)
{
    portENTER_CRITICAL(&s_snapshot_lock);
    s_operational = operational;
    s_periodic_started_at_us =
        operational ? esp_timer_get_time() : 0LL;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

static esp_err_t stop_periodic_measurement(void)
{
    ESP_RETURN_ON_ERROR(
        send_command(SCD4X_CMD_STOP_PERIODIC_MEASUREMENT), TAG,
        "Unable to stop periodic measurement for SCD40 control action");
    vTaskDelay(pdMS_TO_TICKS(SCD4X_STOP_DELAY_MS));
    return ESP_OK;
}

static esp_err_t perform_forced_calibration(uint16_t reference_ppm,
                                            int16_t *correction_ppm)
{
    ESP_RETURN_ON_FALSE(correction_ppm != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Missing forced-calibration result buffer");
    ESP_RETURN_ON_ERROR(
        send_command_with_word(SCD4X_CMD_PERFORM_FORCED_RECALIBRATION,
                               reference_ppm),
        TAG, "Unable to start forced calibration at %u ppm", reference_ppm);
    vTaskDelay(pdMS_TO_TICKS(SCD4X_FORCED_RECALIBRATION_DELAY_MS));

    uint8_t response[SCD4X_WORD_RESPONSE_SIZE];
    ESP_RETURN_ON_ERROR(
        i2c_master_receive(s_device, response, sizeof(response),
                           S1_PRO_I2C_TIMEOUT_MS),
        TAG, "Unable to read forced-calibration result");
    ESP_RETURN_ON_FALSE(
        scd4x_decode_forced_recalibration_response(response, correction_ppm),
        ESP_ERR_INVALID_RESPONSE, TAG,
        "SCD40 rejected forced calibration or returned invalid CRC");
    return ESP_OK;
}

static esp_err_t set_and_persist_temperature_offset(
    uint16_t requested_offset_centi_c, uint16_t *confirmed_offset_centi_c)
{
    ESP_RETURN_ON_FALSE(confirmed_offset_centi_c != NULL,
                        ESP_ERR_INVALID_ARG, TAG,
                        "Missing confirmed temperature-offset buffer");
    const uint16_t raw_offset =
        scd4x_temperature_offset_centi_c_to_raw(requested_offset_centi_c);
    ESP_RETURN_ON_ERROR(
        send_command_with_word(SCD4X_CMD_SET_TEMPERATURE_OFFSET, raw_offset),
        TAG, "Unable to set SCD40 Temp Offset to %.2f C",
        requested_offset_centi_c / 100.0);
    vTaskDelay(pdMS_TO_TICKS(SCD4X_SETTING_DELAY_MS));
    ESP_RETURN_ON_ERROR(send_command(SCD4X_CMD_PERSIST_SETTINGS), TAG,
                        "Unable to persist SCD40 Temp Offset");
    vTaskDelay(pdMS_TO_TICKS(SCD4X_PERSIST_SETTINGS_DELAY_MS));
    ESP_RETURN_ON_ERROR(read_temperature_offset(confirmed_offset_centi_c), TAG,
                        "Unable to confirm SCD40 Temp Offset");
    return ESP_OK;
}

static esp_err_t execute_control_request(
    const scd40_control_request_t *request)
{
    ESP_RETURN_ON_FALSE(request != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Missing SCD40 control request");
    set_operational(false);

    esp_err_t operation_status = stop_periodic_measurement();
    const bool sensor_is_idle = operation_status == ESP_OK;
    int16_t correction_ppm = 0;
    uint16_t confirmed_temperature_offset_centi_c = 0U;

    if (operation_status == ESP_OK &&
        request->type == SCD40_CONTROL_FORCED_CALIBRATION) {
        operation_status = perform_forced_calibration(
            request->reference_ppm, &correction_ppm);
    } else if (operation_status == ESP_OK &&
               request->type == SCD40_CONTROL_FACTORY_RESET) {
        operation_status = send_command(SCD4X_CMD_PERFORM_FACTORY_RESET);
        if (operation_status == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(SCD4X_FACTORY_RESET_DELAY_MS));
            operation_status = read_temperature_offset(
                &confirmed_temperature_offset_centi_c);
            if (operation_status == ESP_OK) {
                store_temperature_offset(
                    confirmed_temperature_offset_centi_c);
            }
        }
    } else if (operation_status == ESP_OK &&
               request->type ==
                   SCD40_CONTROL_SET_TEMPERATURE_OFFSET) {
        operation_status = set_and_persist_temperature_offset(
            request->temperature_offset_centi_c,
            &confirmed_temperature_offset_centi_c);
        if (operation_status == ESP_OK) {
            store_temperature_offset(confirmed_temperature_offset_centi_c);
        }
    }

    esp_err_t resume_status = ESP_OK;
    if (sensor_is_idle) {
        resume_status =
            send_command(SCD4X_CMD_START_PERIODIC_MEASUREMENT);
        if (resume_status == ESP_OK) {
            set_operational(true);
        }
    }

    if (operation_status != ESP_OK) {
        return operation_status;
    }
    if (resume_status != ESP_OK) {
        return resume_status;
    }

    return ESP_OK;
}

static void store_measurement(const scd4x_measurement_t *measurement)
{
    portENTER_CRITICAL(&s_snapshot_lock);
    s_snapshot.received_at_us = esp_timer_get_time();
    s_snapshot.co2_ppm = measurement->co2_ppm;
    s_snapshot.temperature_centi_c = measurement->temperature_centi_c;
    s_snapshot.humidity_centi_percent = measurement->humidity_centi_percent;
    s_snapshot.valid = true;
    ++s_snapshot.measurement_count;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

static bool detect_sensor(void)
{
    const esp_err_t status = i2c_master_probe(
        s1_pro_i2c_bus_get(), S1_PRO_SCD40_I2C_ADDRESS,
        S1_PRO_I2C_TIMEOUT_MS);
    return status == ESP_OK;
}

static void scd40_task(void *argument)
{
    (void)argument;
    bool device_added = false;

    while (true) {
        if (!detect_sensor()) {
            vTaskDelay(pdMS_TO_TICKS(S1_PRO_SCD40_DETECT_RETRY_MS));
            continue;
        }
        if (!device_added) {
            const esp_err_t add_status = s1_pro_i2c_add_device(
                S1_PRO_SCD40_I2C_ADDRESS, &s_device);
            if (add_status != ESP_OK) {
                record_error();
                vTaskDelay(pdMS_TO_TICKS(S1_PRO_SCD40_DETECT_RETRY_MS));
                continue;
            }
            device_added = true;
        }

        if (initialize_sensor() != ESP_OK) {
            set_operational(false);
            record_error();
            vTaskDelay(pdMS_TO_TICKS(S1_PRO_SCD40_DETECT_RETRY_MS));
            continue;
        }
        set_operational(true);

        uint8_t consecutive_errors = 0;
        while (consecutive_errors < SCD4X_MAX_CONSECUTIVE_ERRORS) {
            scd40_control_request_t request;
            if (xQueueReceive(s_control_queue, &request,
                              pdMS_TO_TICKS(S1_PRO_SCD40_POLL_PERIOD_MS)) ==
                pdTRUE) {
                const esp_err_t control_status =
                    execute_control_request(&request);
                if (control_status == ESP_OK) {
                    consecutive_errors = 0;
                } else {
                    record_error();
                    consecutive_errors = SCD4X_MAX_CONSECUTIVE_ERRORS;
                }
                continue;
            }

            bool ready = false;
            esp_err_t status = get_data_ready(&ready);
            if (status == ESP_OK && ready) {
                scd4x_measurement_t measurement;
                status = read_measurement(&measurement);
                if (status == ESP_OK) {
                    store_measurement(&measurement);
                }
            }
            if (status != ESP_OK) {
                ++consecutive_errors;
                record_error();
            } else {
                consecutive_errors = 0;
            }
        }
        set_operational(false);
    }
}

void scd40_sensor_get_snapshot(scd40_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_snapshot_lock);
    *snapshot = s_snapshot;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

void scd40_sensor_start(void)
{
    s_control_queue = xQueueCreate(S1_PRO_SCD40_CONTROL_QUEUE_LENGTH,
                                   sizeof(scd40_control_request_t));
    ESP_ERROR_CHECK(s_control_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    const BaseType_t created =
        xTaskCreate(scd40_task, "scd40", 4096, NULL, 4, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

static bool queue_control_request(scd40_control_type_t type,
                                  uint16_t reference_ppm,
                                  uint16_t temperature_offset_centi_c)
{
    if (s_control_queue == NULL) {
        return false;
    }

    bool operational = false;
    int64_t periodic_started_at_us = 0LL;
    bool has_measurement = false;
    portENTER_CRITICAL(&s_snapshot_lock);
    operational = s_operational;
    periodic_started_at_us = s_periodic_started_at_us;
    has_measurement = s_snapshot.valid;
    portEXIT_CRITICAL(&s_snapshot_lock);

    if (!operational) {
        return false;
    }
    if (type == SCD40_CONTROL_FORCED_CALIBRATION &&
        (!has_measurement ||
         esp_timer_get_time() - periodic_started_at_us <
             (int64_t)SCD4X_FORCED_RECALIBRATION_MIN_RUNTIME_MS * 1000LL)) {
        return false;
    }

    const scd40_control_request_t request = {
        .type = type,
        .reference_ppm = reference_ppm,
        .temperature_offset_centi_c = temperature_offset_centi_c,
    };
    return xQueueSend(s_control_queue, &request, 0) == pdTRUE;
}

bool scd40_sensor_request_forced_calibration(uint16_t reference_ppm)
{
    if (reference_ppm < S1_PRO_SCD40_MIN_CALIBRATION_REFERENCE_PPM ||
        reference_ppm > S1_PRO_SCD40_MAX_CALIBRATION_REFERENCE_PPM) {
        return false;
    }
    return queue_control_request(SCD40_CONTROL_FORCED_CALIBRATION,
                                 reference_ppm, 0U);
}

bool scd40_sensor_request_factory_reset(void)
{
    return queue_control_request(SCD40_CONTROL_FACTORY_RESET, 0U, 0U);
}

bool scd40_sensor_request_temperature_offset(uint16_t offset_centi_c)
{
    if (offset_centi_c >
        S1_PRO_SCD40_MAX_TEMPERATURE_OFFSET_CENTI_C) {
        return false;
    }

    bool already_confirmed = false;
    portENTER_CRITICAL(&s_snapshot_lock);
    already_confirmed = s_snapshot.temperature_offset_valid &&
                        s_snapshot.temperature_offset_centi_c ==
                            offset_centi_c;
    portEXIT_CRITICAL(&s_snapshot_lock);
    if (already_confirmed) {
        return true;
    }
    return queue_control_request(SCD40_CONTROL_SET_TEMPERATURE_OFFSET, 0U,
                                 offset_centi_c);
}
