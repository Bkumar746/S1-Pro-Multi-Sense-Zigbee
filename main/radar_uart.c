#include "radar_uart.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "ld2450_command_protocol.h"
#include "radar_settings.h"
#include "s1_pro_config.h"

static const char *TAG = "ld2450_uart";
static portMUX_TYPE s_snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static radar_snapshot_t s_snapshot;
static QueueHandle_t s_control_queue;

typedef enum {
    RADAR_CONTROL_BLUETOOTH,
    RADAR_CONTROL_SINGLE_TARGET,
    RADAR_CONTROL_RESTART,
    RADAR_CONTROL_FACTORY_RESET,
} radar_control_type_t;

typedef struct {
    radar_control_type_t type;
    bool enabled;
} radar_control_request_t;

static const uint8_t s_command_header[LD2450_COMMAND_HEADER_SIZE] = {
    0xFD, 0xFC, 0xFB, 0xFA};

static void parser_frame_callback(const ld2450_frame_t *frame, void *context);

static void increment_command_errors(void)
{
    portENTER_CRITICAL(&s_snapshot_lock);
    ++s_snapshot.command_errors;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

static void mark_radar_restarting(void)
{
    portENTER_CRITICAL(&s_snapshot_lock);
    s_snapshot.valid = false;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

static void reset_command_frame(uint8_t *frame_length, size_t *expected_length,
                                uint8_t byte)
{
    *frame_length = byte == s_command_header[0] ? 1U : 0U;
    *expected_length = 0U;
}

static esp_err_t read_command_ack(uint16_t expected_command,
                                  uint8_t *return_value,
                                  size_t return_value_capacity,
                                  size_t *return_value_length)
{
    uint8_t frame[LD2450_COMMAND_MAX_FRAME_SIZE] = {0};
    uint8_t frame_length = 0U;
    size_t expected_length = 0U;
    const int64_t deadline_us =
        esp_timer_get_time() +
        (int64_t)S1_PRO_LD2450_COMMAND_TIMEOUT_MS * 1000LL;

    if (return_value_length != NULL) {
        *return_value_length = 0U;
    }

    while (esp_timer_get_time() < deadline_us) {
        uint8_t byte = 0U;
        const int bytes_read = uart_read_bytes(
            S1_PRO_LD2450_UART, &byte, 1U, pdMS_TO_TICKS(20));
        if (bytes_read < 0) {
            return ESP_FAIL;
        }
        if (bytes_read == 0) {
            continue;
        }

        if (frame_length < LD2450_COMMAND_HEADER_SIZE) {
            if (byte == s_command_header[frame_length]) {
                frame[frame_length++] = byte;
            } else {
                reset_command_frame(&frame_length, &expected_length, byte);
                if (frame_length == 1U) {
                    frame[0] = byte;
                }
            }
            continue;
        }

        if ((size_t)frame_length >= sizeof(frame)) {
            reset_command_frame(&frame_length, &expected_length, byte);
            continue;
        }
        frame[frame_length++] = byte;

        if (frame_length == 6U) {
            expected_length = ld2450_command_frame_size_from_prefix(
                frame, frame_length);
            if (expected_length == 0U || expected_length > sizeof(frame)) {
                reset_command_frame(&frame_length, &expected_length, byte);
            }
            continue;
        }
        if (expected_length == 0U || frame_length < expected_length) {
            continue;
        }

        const uint8_t *ack_return_value = NULL;
        size_t ack_return_length = 0U;
        const ld2450_ack_result_t result = ld2450_parse_ack_frame(
            frame, frame_length, expected_command, &ack_return_value,
            &ack_return_length);
        if (result == LD2450_ACK_WRONG_COMMAND ||
            result == LD2450_ACK_INVALID_FRAME) {
            reset_command_frame(&frame_length, &expected_length, byte);
            continue;
        }
        if (result == LD2450_ACK_COMMAND_FAILED) {
            return ESP_FAIL;
        }
        if (return_value != NULL &&
            ack_return_length > return_value_capacity) {
            return ESP_ERR_INVALID_SIZE;
        }
        if (return_value != NULL && ack_return_length != 0U) {
            memcpy(return_value, ack_return_value, ack_return_length);
        }
        if (return_value_length != NULL) {
            *return_value_length = ack_return_length;
        }
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t send_command(uint16_t command, const uint8_t *value,
                              size_t value_length, uint8_t *return_value,
                              size_t return_value_capacity,
                              size_t *return_value_length)
{
    uint8_t frame[LD2450_COMMAND_MAX_FRAME_SIZE] = {0};
    const size_t frame_length = ld2450_build_command_frame(
        command, value, value_length, frame, sizeof(frame));
    ESP_RETURN_ON_FALSE(frame_length != 0U, ESP_ERR_INVALID_ARG, TAG,
                        "Unable to build LD2450 command 0x%04x", command);

    const int written =
        uart_write_bytes(S1_PRO_LD2450_UART, frame, frame_length);
    ESP_RETURN_ON_FALSE(written == (int)frame_length, ESP_FAIL, TAG,
                        "LD2450 command 0x%04x write failed", command);
    ESP_RETURN_ON_ERROR(
        uart_wait_tx_done(S1_PRO_LD2450_UART, pdMS_TO_TICKS(100)), TAG,
        "LD2450 command 0x%04x TX timeout", command);
    return read_command_ack(command, return_value, return_value_capacity,
                            return_value_length);
}

static esp_err_t begin_configuration(void)
{
    const uint8_t enable_value[] = {0x01, 0x00};
    ESP_RETURN_ON_ERROR(uart_flush_input(S1_PRO_LD2450_UART), TAG,
                        "Unable to flush LD2450 UART before command");
    return send_command(LD2450_COMMAND_ENABLE_CONFIGURATION, enable_value,
                        sizeof(enable_value), NULL, 0U, NULL);
}

static esp_err_t end_configuration(void)
{
    return send_command(LD2450_COMMAND_END_CONFIGURATION, NULL, 0U, NULL, 0U,
                        NULL);
}

static esp_err_t read_tracking_mode_in_configuration(bool *single_target)
{
    ESP_RETURN_ON_FALSE(single_target != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Missing tracking-mode destination");
    uint8_t mode_value[2] = {0};
    size_t mode_length = 0U;
    ESP_RETURN_ON_ERROR(
        send_command(LD2450_COMMAND_QUERY_TRACKING_MODE, NULL, 0U, mode_value,
                     sizeof(mode_value), &mode_length),
        TAG, "Unable to query LD2450 tracking mode");
    ESP_RETURN_ON_FALSE(mode_length == sizeof(mode_value),
                        ESP_ERR_INVALID_RESPONSE, TAG,
                        "Invalid LD2450 tracking-mode response length: %u",
                        (unsigned)mode_length);
    const uint16_t returned_mode =
        (uint16_t)mode_value[0] | ((uint16_t)mode_value[1] << 8U);
    ESP_RETURN_ON_FALSE(returned_mode == 1U || returned_mode == 2U,
                        ESP_ERR_INVALID_RESPONSE, TAG,
                        "Invalid LD2450 tracking mode: %u", returned_mode);
    *single_target = returned_mode == 1U;
    return ESP_OK;
}

static esp_err_t set_tracking_mode(bool single_target)
{
    ESP_RETURN_ON_ERROR(begin_configuration(), TAG,
                        "Unable to enter LD2450 configuration mode");

    esp_err_t status = send_command(
        single_target ? LD2450_COMMAND_SINGLE_TARGET
                      : LD2450_COMMAND_MULTI_TARGET,
        NULL, 0U, NULL, 0U, NULL);
    if (status == ESP_OK) {
        bool confirmed_single_target = false;
        status =
            read_tracking_mode_in_configuration(&confirmed_single_target);
        if (status == ESP_OK &&
            confirmed_single_target != single_target) {
            status = ESP_ERR_INVALID_RESPONSE;
        }
    }

    const esp_err_t end_status = end_configuration();
    return status == ESP_OK ? end_status : status;
}

static esp_err_t query_tracking_mode(bool *single_target)
{
    ESP_RETURN_ON_ERROR(begin_configuration(), TAG,
                        "Unable to enter LD2450 configuration mode");
    const esp_err_t status =
        read_tracking_mode_in_configuration(single_target);
    const esp_err_t end_status = end_configuration();
    return status == ESP_OK ? end_status : status;
}

static esp_err_t set_bluetooth(bool enabled)
{
    ESP_RETURN_ON_ERROR(begin_configuration(), TAG,
                        "Unable to enter LD2450 configuration mode");
    const uint8_t bluetooth_value[] = {enabled ? 0x01U : 0x00U, 0x00};
    esp_err_t status = send_command(LD2450_COMMAND_BLUETOOTH,
                                    bluetooth_value,
                                    sizeof(bluetooth_value), NULL, 0U, NULL);
    if (status == ESP_OK) {
        status = send_command(LD2450_COMMAND_RESTART, NULL, 0U, NULL, 0U,
                              NULL);
        if (status == ESP_OK) {
            mark_radar_restarting();
        }
    } else {
        (void)end_configuration();
    }
    return status;
}

static esp_err_t restart_module(void)
{
    ESP_RETURN_ON_ERROR(begin_configuration(), TAG,
                        "Unable to enter LD2450 configuration mode");
    const esp_err_t status = send_command(LD2450_COMMAND_RESTART, NULL, 0U,
                                          NULL, 0U, NULL);
    if (status == ESP_OK) {
        mark_radar_restarting();
    } else {
        (void)end_configuration();
    }
    return status;
}

static esp_err_t factory_reset_module(void)
{
    ESP_RETURN_ON_ERROR(begin_configuration(), TAG,
                        "Unable to enter LD2450 configuration mode");
    esp_err_t status = send_command(LD2450_COMMAND_FACTORY_RESET, NULL, 0U,
                                    NULL, 0U, NULL);
    if (status == ESP_OK) {
        status = send_command(LD2450_COMMAND_RESTART, NULL, 0U, NULL, 0U,
                              NULL);
        if (status == ESP_OK) {
            mark_radar_restarting();
        }
    } else {
        (void)end_configuration();
    }
    return status;
}

static void reset_parser_preserving_statistics(ld2450_parser_t *parser)
{
    const uint32_t valid_frames = parser->valid_frames;
    const uint32_t framing_errors = parser->framing_errors;
    const uint32_t discarded_bytes = parser->discarded_bytes;
    ld2450_parser_init(parser, parser_frame_callback, parser);
    parser->valid_frames = valid_frames;
    parser->framing_errors = framing_errors;
    parser->discarded_bytes = discarded_bytes;
}

static void handle_control_request(const radar_control_request_t *request,
                                   ld2450_parser_t *parser)
{
    esp_err_t status = ESP_ERR_INVALID_ARG;
    switch (request->type) {
    case RADAR_CONTROL_BLUETOOTH:
        status = set_bluetooth(request->enabled);
        if (status == ESP_OK) {
            radar_settings_set_bluetooth_enabled(request->enabled);
        }
        break;
    case RADAR_CONTROL_SINGLE_TARGET:
        status = set_tracking_mode(request->enabled);
        if (status == ESP_OK) {
            radar_settings_set_single_target(request->enabled);
        }
        break;
    case RADAR_CONTROL_RESTART:
        status = restart_module();
        break;
    case RADAR_CONTROL_FACTORY_RESET:
        status = factory_reset_module();
        if (status == ESP_OK) {
            radar_settings_set_bluetooth_enabled(true);
            radar_settings_set_single_target(false);
        }
        break;
    }

    reset_parser_preserving_statistics(parser);
    if (status != ESP_OK) {
        increment_command_errors();
    }
}

static void synchronize_tracking_mode(ld2450_parser_t *parser)
{
    bool single_target = false;
    const esp_err_t status = query_tracking_mode(&single_target);
    reset_parser_preserving_statistics(parser);
    if (status != ESP_OK) {
        increment_command_errors();
        return;
    }

    radar_settings_set_single_target(single_target);
}

static bool synchronize_bluetooth_preference(ld2450_parser_t *parser)
{
    const bool enabled = radar_settings_get_bluetooth_enabled();
    const esp_err_t status = set_bluetooth(enabled);
    reset_parser_preserving_statistics(parser);
    if (status != ESP_OK) {
        increment_command_errors();
        return false;
    }

    return true;
}

static bool queue_control_request(radar_control_type_t type, bool enabled)
{
    if (s_control_queue == NULL) {
        return false;
    }
    const radar_control_request_t request = {
        .type = type,
        .enabled = enabled,
    };
    return xQueueSend(s_control_queue, &request, 0) == pdTRUE;
}

static void parser_frame_callback(const ld2450_frame_t *frame, void *context)
{
    ld2450_parser_t *parser = context;

    portENTER_CRITICAL(&s_snapshot_lock);
    s_snapshot.frame = *frame;
    s_snapshot.received_at_us = esp_timer_get_time();
    s_snapshot.valid_frames = parser->valid_frames;
    s_snapshot.framing_errors = parser->framing_errors;
    s_snapshot.valid = true;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

void radar_uart_get_snapshot(radar_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_snapshot_lock);
    *snapshot = s_snapshot;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

static void radar_uart_task(void *argument)
{
    (void)argument;
    uint8_t read_buffer[256];
    ld2450_parser_t parser;
    TickType_t last_status_sync = xTaskGetTickCount();
    TickType_t last_bluetooth_sync_attempt = 0;
    bool bluetooth_preference_synchronized = false;
    bool first_bluetooth_sync_attempt = true;

    ld2450_parser_init(&parser, parser_frame_callback, &parser);
    vTaskDelay(pdMS_TO_TICKS(S1_PRO_LD2450_STARTUP_SYNC_DELAY_MS));

    while (true) {
        radar_control_request_t control_request;
        if (xQueueReceive(s_control_queue, &control_request, 0) == pdTRUE) {
            handle_control_request(&control_request, &parser);
            continue;
        }

        const TickType_t before_read = xTaskGetTickCount();
        if (!bluetooth_preference_synchronized &&
            (first_bluetooth_sync_attempt ||
             before_read - last_bluetooth_sync_attempt >=
                 pdMS_TO_TICKS(S1_PRO_LD2450_STARTUP_SYNC_RETRY_MS))) {
            first_bluetooth_sync_attempt = false;
            last_bluetooth_sync_attempt = before_read;
            bluetooth_preference_synchronized =
                synchronize_bluetooth_preference(&parser);
            last_status_sync = xTaskGetTickCount();
            continue;
        }

        if (before_read - last_status_sync >=
            pdMS_TO_TICKS(S1_PRO_LD2450_STATUS_SYNC_INTERVAL_MS)) {
            synchronize_tracking_mode(&parser);
            last_status_sync = xTaskGetTickCount();
            continue;
        }

        const int bytes_read = uart_read_bytes(S1_PRO_LD2450_UART, read_buffer, sizeof(read_buffer), pdMS_TO_TICKS(100));
        if (bytes_read > 0) {
            ld2450_parser_feed(&parser, read_buffer, (size_t)bytes_read);
            portENTER_CRITICAL(&s_snapshot_lock);
            s_snapshot.valid_frames = parser.valid_frames;
            s_snapshot.framing_errors = parser.framing_errors;
            portEXIT_CRITICAL(&s_snapshot_lock);
        } else if (bytes_read < 0) {
            portENTER_CRITICAL(&s_snapshot_lock);
            ++s_snapshot.uart_errors;
            portEXIT_CRITICAL(&s_snapshot_lock);
        }

    }
}

void radar_uart_start(void)
{
    const uart_config_t uart_config = {
        .baud_rate = S1_PRO_LD2450_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(S1_PRO_LD2450_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(S1_PRO_LD2450_UART, S1_PRO_LD2450_TX_GPIO, S1_PRO_LD2450_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(S1_PRO_LD2450_UART, 2048, 0, 0, NULL, 0));
    s_control_queue = xQueueCreate(S1_PRO_LD2450_CONTROL_QUEUE_LENGTH,
                                   sizeof(radar_control_request_t));
    ESP_ERROR_CHECK(s_control_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    BaseType_t created = xTaskCreate(radar_uart_task, "ld2450_uart", 4096, NULL, 6, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

bool radar_uart_request_bluetooth(bool enabled)
{
    return queue_control_request(RADAR_CONTROL_BLUETOOTH, enabled);
}

bool radar_uart_request_single_target(bool enabled)
{
    return queue_control_request(RADAR_CONTROL_SINGLE_TARGET, enabled);
}

bool radar_uart_request_restart(void)
{
    return queue_control_request(RADAR_CONTROL_RESTART, false);
}

bool radar_uart_request_factory_reset(void)
{
    return queue_control_request(RADAR_CONTROL_FACTORY_RESET, false);
}
