#include "zigbee_device.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "driver/temperature_sensor.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "esp_zigbee_attribute.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_endpoint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "zcl/esp_zigbee_zcl_color_control.h"
#include "zcl/esp_zigbee_zcl_level.h"
#include "zcl/esp_zigbee_zcl_on_off.h"
#include "actuator_state.h"
#include "bme688_sensor.h"
#include "buzzer.h"
#include "ltr390_sensor.h"
#include "presence_delay.h"
#include "radar_settings.h"
#include "radar_transform.h"
#include "radar_uart.h"
#include "radar_zones.h"
#include "rgb_led.h"
#include "s1_pro_config.h"
#include "scd40_sensor.h"
#include "scd40_settings.h"
#include "system_control.h"

static const char *TAG = "zigbee";
static volatile bool s_zigbee_ready;
static volatile bool s_joined;
static uint32_t s_last_bme_measurement_count;
static bool s_bme_state_published;
static uint8_t s_last_bsec_accuracy;
static uint8_t s_last_iaq_classification;
static bool s_radar_state_published;
static uint8_t s_last_target_count;
static bool s_last_any_presence;
static bool s_last_any_movement;
static bool s_detection_range_published;
static uint16_t s_last_detection_range_cm;
static bool s_movement_threshold_published;
static uint16_t s_last_movement_threshold_cms;
static bool s_presence_delay_published;
static uint16_t s_last_presence_delay_s;
static presence_delay_state_t s_presence_delay_state;
static bool s_radar_controls_published;
static bool s_last_flip_y_axis;
static bool s_last_bluetooth_enabled;
static bool s_last_single_target;
static ld2450_target_t s_last_targets[LD2450_TARGET_COUNT];
static radar_zones_config_t s_last_zones_config;
static bool s_zones_config_published;
static presence_delay_state_t s_zone_presence_delay_states[RADAR_ZONE_COUNT];
static radar_zone_result_t s_last_zone_results[RADAR_ZONE_COUNT];
static uint16_t s_pending_radar_reports[128];
static size_t s_pending_radar_report_count;
static uint16_t s_pending_bme_reports[16];
static size_t s_pending_bme_report_count;
static uint32_t s_last_scd40_measurement_count;
static bool s_scd40_state_published;
static bool s_scd40_temperature_offset_published;
static uint16_t s_last_scd40_temperature_offset_centi_c;
static uint16_t s_pending_scd40_reports[8];
static size_t s_pending_scd40_report_count;
static uint32_t s_last_ltr390_measurement_count;
static bool s_ltr390_state_published;
static uint16_t s_pending_ltr390_reports[8];
static size_t s_pending_ltr390_report_count;
static temperature_sensor_handle_t s_esp32_temperature_sensor;
static bool s_esp32_temperature_published;
static int16_t s_last_esp32_temperature_centi_c;
static bool s_esp32_status_published;
static bool s_last_esp32_connected;
static uint16_t s_pending_esp32_reports[2];
static size_t s_pending_esp32_report_count;
static uint8_t s_software_build_id[17];

#define S1_PRO_RADAR_REPORTS_PER_PUBLISH 12U
#define S1_PRO_BME_REPORTS_PER_PUBLISH 1U
#define S1_PRO_SCD40_REPORTS_PER_PUBLISH 1U
#define S1_PRO_LTR390_REPORTS_PER_PUBLISH 1U
#define S1_PRO_ESP32_REPORTS_PER_PUBLISH 1U

static void set_joined(bool joined)
{
    s_joined = joined;
    rgb_led_set_zigbee_connected(joined);
}

static void commissioning_retry_callback(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG,
                        "Failed to start BDB commissioning mode 0x%02x", mode_mask);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *signal = signal_struct->p_app_signal;
    const esp_err_t status = signal_struct->esp_err_status;
    const esp_zb_app_signal_type_t signal_type = *signal;

    switch (signal_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (status == ESP_OK) {
            if (esp_zb_bdb_is_factory_new()) {
                set_joined(false);
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                set_joined(true);
            }
        } else {
            esp_zb_scheduler_alarm((esp_zb_callback_t)commissioning_retry_callback,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (status == ESP_OK) {
            set_joined(true);
        } else {
            set_joined(false);
            esp_zb_scheduler_alarm((esp_zb_callback_t)commissioning_retry_callback,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 5000);
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
    case ESP_ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
        set_joined(false);
        break;
    case ESP_ZB_BDB_SIGNAL_TC_REJOIN_DONE:
        set_joined(status == ESP_OK);
        break;
    default:
        break;
    }
}

static esp_err_t add_custom_attribute(esp_zb_attribute_list_t *cluster, uint16_t id, uint8_t type, void *initial_value,
                                      bool reporting)
{
    const uint8_t access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY |
                           (reporting ? ESP_ZB_ZCL_ATTR_ACCESS_REPORTING : 0);
    return esp_zb_custom_cluster_add_custom_attr(cluster, id, type, access, initial_value);
}

static esp_err_t add_custom_writable_attribute(esp_zb_attribute_list_t *cluster,
                                               uint16_t id, uint8_t type,
                                               void *initial_value,
                                               bool reporting)
{
    const uint8_t access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE |
                           (reporting ? ESP_ZB_ZCL_ATTR_ACCESS_REPORTING : 0);
    return esp_zb_custom_cluster_add_custom_attr(cluster, id, type, access,
                                                 initial_value);
}

static esp_zb_ep_list_t *create_endpoint(void)
{
    static int16_t initial_s16;
    static uint8_t initial_u8;
    static uint16_t initial_u16;
    static uint32_t initial_u32;
    static bool initial_bool;
    static bool initial_buzzer_power;
    static uint16_t initial_detection_range_cm;
    static uint16_t initial_movement_threshold_cms;
    static uint16_t initial_presence_delay_s;
    static bool initial_flip_y_axis;
    static bool initial_bluetooth_enabled;
    static bool initial_single_target;
    static uint16_t initial_bme_temperature_offset_centi_c;
    static uint16_t initial_scd40_calibration_reference_ppm;
    static uint16_t initial_scd40_temperature_offset_centi_c;
    static radar_zones_config_t initial_zones;
    actuator_state_snapshot_t output_state;
    actuator_state_get(&output_state);
    initial_buzzer_power = output_state.buzzer_power;
    initial_detection_range_cm = radar_settings_get_detection_range_cm();
    initial_movement_threshold_cms =
        radar_settings_get_movement_threshold_cms();
    initial_presence_delay_s = radar_settings_get_presence_delay_s();
    initial_flip_y_axis = radar_settings_get_flip_y_axis();
    initial_bluetooth_enabled = radar_settings_get_bluetooth_enabled();
    initial_single_target = radar_settings_get_single_target();
    radar_settings_get_zones(&initial_zones);
    s_last_zones_config = initial_zones;
    s_zones_config_published = true;
    initial_bme_temperature_offset_centi_c =
        bme688_sensor_get_temperature_offset_centi_c();
    initial_scd40_calibration_reference_ppm =
        scd40_settings_get_calibration_reference_ppm();
    initial_scd40_temperature_offset_centi_c =
        S1_PRO_SCD40_DEFAULT_TEMPERATURE_OFFSET_CENTI_C;
    const esp_zb_basic_cluster_cfg_t basic_config = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01,
    };
    esp_zb_color_dimmable_light_cfg_t light_config =
        ESP_ZB_DEFAULT_COLOR_DIMMABLE_LIGHT_CONFIG();
    light_config.on_off_cfg.on_off = output_state.led_power;
    light_config.level_cfg.current_level = output_state.led_level;
    light_config.color_cfg.current_x = output_state.led_x;
    light_config.color_cfg.current_y = output_state.led_y;

    esp_zb_cluster_list_t *clusters = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create((esp_zb_basic_cluster_cfg_t *)&basic_config);
    const char *software_version = esp_app_get_description()->version;
    const size_t software_version_length =
        strnlen(software_version, sizeof(s_software_build_id) - 1U);
    s_software_build_id[0] = (uint8_t)software_version_length;
    memcpy(&s_software_build_id[1], software_version, software_version_length);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                                  (void *)S1_PRO_MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                                  (void *)S1_PRO_MODEL_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
        basic, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, s_software_build_id));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(clusters, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_groups_cluster(
        clusters, esp_zb_groups_cluster_create(&light_config.groups_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_scenes_cluster(
        clusters, esp_zb_scenes_cluster_create(&light_config.scenes_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(
        clusters, esp_zb_on_off_cluster_create(&light_config.on_off_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_level_cluster(
        clusters, esp_zb_level_cluster_create(&light_config.level_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_color_control_cluster(
        clusters, esp_zb_color_control_cluster_create(&light_config.color_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_attribute_list_t *custom = esp_zb_zcl_attr_list_create(S1_PRO_CUSTOM_CLUSTER_ID);
    for (uint8_t target = 0; target < 3; ++target) {
        ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_TARGET_X(target), ESP_ZB_ZCL_ATTR_TYPE_S16,
                                              &initial_s16, true));
        ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_TARGET_Y(target), ESP_ZB_ZCL_ATTR_TYPE_S16,
                                              &initial_s16, true));
        ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_TARGET_DISTANCE(target), ESP_ZB_ZCL_ATTR_TYPE_U16,
                                              &initial_u16, true));
        ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_TARGET_SPEED(target), ESP_ZB_ZCL_ATTR_TYPE_S16,
                                              &initial_s16, true));
    }
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_ALL_TARGETS_COUNT, ESP_ZB_ZCL_ATTR_TYPE_U8,
                                          &initial_u8, true));
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_ANY_PRESENCE, ESP_ZB_ZCL_ATTR_TYPE_BOOL,
                                          &initial_bool, true));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_DETECTION_RANGE, ESP_ZB_ZCL_ATTR_TYPE_U16,
        &initial_detection_range_cm, true));
    ESP_ERROR_CHECK(add_custom_attribute(
        custom, S1_PRO_ATTR_ANY_MOVEMENT, ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        &initial_bool, true));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_ANY_MOVEMENT_THRESHOLD,
        ESP_ZB_ZCL_ATTR_TYPE_U16, &initial_movement_threshold_cms, true));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_ANY_PRESENCE_DELAY, ESP_ZB_ZCL_ATTR_TYPE_U16,
        &initial_presence_delay_s, true));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_RADAR_FLIP_Y_AXIS, ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        &initial_flip_y_axis, true));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_RADAR_BLUETOOTH, ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        &initial_bluetooth_enabled, true));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_RADAR_SINGLE_TARGET, ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        &initial_single_target, true));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_RADAR_RESTART_MODULE, ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        &initial_bool, false));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_RADAR_FACTORY_RESET, ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        &initial_bool, false));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_ESP32_RESTART_MODULE, ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        &initial_bool, false));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_ESP32_FACTORY_RESET, ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        &initial_bool, false));
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_BME_TEMPERATURE, ESP_ZB_ZCL_ATTR_TYPE_S16,
                                          &initial_s16, true));
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_BME_HUMIDITY, ESP_ZB_ZCL_ATTR_TYPE_U16,
                                          &initial_u16, true));
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_BME_PRESSURE, ESP_ZB_ZCL_ATTR_TYPE_U32,
                                          &initial_u32, true));
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_BME_GAS_RESISTANCE, ESP_ZB_ZCL_ATTR_TYPE_U32,
                                          &initial_u32, true));
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_BME_CO2_EQUIVALENT, ESP_ZB_ZCL_ATTR_TYPE_U16,
                                          &initial_u16, true));
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_BME_VOC_EQUIVALENT, ESP_ZB_ZCL_ATTR_TYPE_U16,
                                          &initial_u16, true));
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_BME_BSEC_ACCURACY, ESP_ZB_ZCL_ATTR_TYPE_U8,
                                          &initial_u8, true));
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_BME_IAQ, ESP_ZB_ZCL_ATTR_TYPE_U16,
                                          &initial_u16, true));
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_BME_IAQ_CLASSIFICATION, ESP_ZB_ZCL_ATTR_TYPE_U8,
                                          &initial_u8, true));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_BME_TEMPERATURE_OFFSET,
        ESP_ZB_ZCL_ATTR_TYPE_U16,
        &initial_bme_temperature_offset_centi_c, true));
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_SCD40_CO2, ESP_ZB_ZCL_ATTR_TYPE_U16,
                                          &initial_u16, true));
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_SCD40_TEMPERATURE, ESP_ZB_ZCL_ATTR_TYPE_S16,
                                          &initial_s16, true));
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_SCD40_HUMIDITY, ESP_ZB_ZCL_ATTR_TYPE_U16,
                                          &initial_u16, true));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_SCD40_CALIBRATION_REFERENCE,
        ESP_ZB_ZCL_ATTR_TYPE_U16,
        &initial_scd40_calibration_reference_ppm, true));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_SCD40_FORCED_CALIBRATION,
        ESP_ZB_ZCL_ATTR_TYPE_BOOL, &initial_bool, false));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_SCD40_FACTORY_RESET,
        ESP_ZB_ZCL_ATTR_TYPE_BOOL, &initial_bool, false));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_SCD40_TEMPERATURE_OFFSET,
        ESP_ZB_ZCL_ATTR_TYPE_U16,
        &initial_scd40_temperature_offset_centi_c, true));
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_LTR390_AMBIENT_LIGHT, ESP_ZB_ZCL_ATTR_TYPE_U32,
                                          &initial_u32, true));
    ESP_ERROR_CHECK(add_custom_attribute(custom, S1_PRO_ATTR_LTR390_UV_INDEX, ESP_ZB_ZCL_ATTR_TYPE_U16,
                                          &initial_u16, true));
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_BUZZER_POWER, ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        &initial_buzzer_power, true));
    ESP_ERROR_CHECK(add_custom_attribute(
        custom, S1_PRO_ATTR_ESP32_TEMPERATURE, ESP_ZB_ZCL_ATTR_TYPE_S16,
        &initial_s16, true));
    ESP_ERROR_CHECK(add_custom_attribute(
        custom, S1_PRO_ATTR_ESP32_CONNECTED, ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        &initial_bool, true));
    for (uint8_t point = 0U; point < RADAR_ZONE_MAX_POINTS; ++point) {
        ESP_ERROR_CHECK(add_custom_writable_attribute(
            custom, S1_PRO_ATTR_EXCLUSION_POINT_X(point),
            ESP_ZB_ZCL_ATTR_TYPE_S16,
            &initial_zones.exclusion_points[point].x_cm, false));
        ESP_ERROR_CHECK(add_custom_writable_attribute(
            custom, S1_PRO_ATTR_EXCLUSION_POINT_Y(point),
            ESP_ZB_ZCL_ATTR_TYPE_S16,
            &initial_zones.exclusion_points[point].y_cm, false));
    }
    ESP_ERROR_CHECK(add_custom_writable_attribute(
        custom, S1_PRO_ATTR_EXCLUSION_POINTS_COUNT,
        ESP_ZB_ZCL_ATTR_TYPE_U8, &initial_zones.exclusion_point_count,
        false));
    for (uint8_t zone = 0U; zone < RADAR_ZONE_COUNT; ++zone) {
        for (uint8_t point = 0U; point < RADAR_ZONE_MAX_POINTS; ++point) {
            ESP_ERROR_CHECK(add_custom_writable_attribute(
                custom, S1_PRO_ATTR_ZONE_POINT_X(zone, point),
                ESP_ZB_ZCL_ATTR_TYPE_S16,
                &initial_zones.zones[zone].points[point].x_cm, false));
            ESP_ERROR_CHECK(add_custom_writable_attribute(
                custom, S1_PRO_ATTR_ZONE_POINT_Y(zone, point),
                ESP_ZB_ZCL_ATTR_TYPE_S16,
                &initial_zones.zones[zone].points[point].y_cm, false));
        }
        ESP_ERROR_CHECK(add_custom_writable_attribute(
            custom, S1_PRO_ATTR_ZONE_POINTS_COUNT(zone),
            ESP_ZB_ZCL_ATTR_TYPE_U8,
            &initial_zones.zones[zone].point_count, false));
        ESP_ERROR_CHECK(add_custom_writable_attribute(
            custom, S1_PRO_ATTR_ZONE_MOVEMENT_THRESHOLD(zone),
            ESP_ZB_ZCL_ATTR_TYPE_U16,
            &initial_zones.zones[zone].movement_threshold_cms, false));
        ESP_ERROR_CHECK(add_custom_writable_attribute(
            custom, S1_PRO_ATTR_ZONE_PRESENCE_DELAY(zone),
            ESP_ZB_ZCL_ATTR_TYPE_U16,
            &initial_zones.zones[zone].presence_delay_s, false));
        ESP_ERROR_CHECK(add_custom_attribute(
            custom, S1_PRO_ATTR_ZONE_PRESENCE(zone),
            ESP_ZB_ZCL_ATTR_TYPE_BOOL, &initial_bool, true));
        ESP_ERROR_CHECK(add_custom_attribute(
            custom, S1_PRO_ATTR_ZONE_MOVEMENT(zone),
            ESP_ZB_ZCL_ATTR_TYPE_BOOL, &initial_bool, true));
        ESP_ERROR_CHECK(add_custom_attribute(
            custom, S1_PRO_ATTR_ZONE_TARGET_COUNT(zone),
            ESP_ZB_ZCL_ATTR_TYPE_U8, &initial_u8, true));
    }
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster(clusters, custom, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    const esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = S1_PRO_ENDPOINT_ID,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID,
        .app_device_version = 1,
    };
    esp_zb_ep_list_t *endpoints = esp_zb_ep_list_create();
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(endpoints, clusters, endpoint_config));
    return endpoints;
}

static bool read_u16_attribute(uint16_t cluster_id, uint16_t attribute_id,
                               uint16_t *value)
{
    esp_zb_zcl_attr_t *attribute = esp_zb_zcl_get_attribute(
        S1_PRO_ENDPOINT_ID, cluster_id, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        attribute_id);
    if (attribute == NULL || attribute->data_p == NULL ||
        attribute->type != ESP_ZB_ZCL_ATTR_TYPE_U16) {
        return false;
    }
    *value = *(uint16_t *)attribute->data_p;
    return true;
}

static bool set_attribute(uint16_t id, void *value);

static esp_err_t handle_set_attribute(
    const esp_zb_zcl_set_attr_value_message_t *message)
{
    ESP_RETURN_ON_FALSE(message != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Empty set-attribute message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
                        ESP_ERR_INVALID_RESPONSE, TAG,
                        "Set-attribute status %d", message->info.status);
    if (message->info.dst_endpoint != S1_PRO_ENDPOINT_ID) {
        return ESP_OK;
    }

    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
        message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL &&
        message->attribute.data.value != NULL) {
        rgb_led_set_power(*(bool *)message->attribute.data.value);
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_DETECTION_RANGE &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 &&
        message->attribute.data.value != NULL) {
        const uint16_t range_cm =
            *(uint16_t *)message->attribute.data.value;
        ESP_RETURN_ON_FALSE(
            radar_settings_set_detection_range_cm(range_cm),
            ESP_ERR_INVALID_ARG, TAG,
            "Detection Range %u cm is outside %u-%u cm", range_cm,
            S1_PRO_RADAR_MIN_RANGE_CM, S1_PRO_RADAR_MAX_RANGE_CM);
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_ANY_MOVEMENT_THRESHOLD &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 &&
        message->attribute.data.value != NULL) {
        const uint16_t threshold_cms =
            *(uint16_t *)message->attribute.data.value;
        ESP_RETURN_ON_FALSE(
            radar_settings_set_movement_threshold_cms(threshold_cms),
            ESP_ERR_INVALID_ARG, TAG,
            "Any Movement Threshold %u cm/s is outside %u-%u cm/s",
            threshold_cms, S1_PRO_RADAR_MIN_MOVEMENT_THRESHOLD_CMS,
            S1_PRO_RADAR_MAX_MOVEMENT_THRESHOLD_CMS);
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_ANY_PRESENCE_DELAY &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 &&
        message->attribute.data.value != NULL) {
        const uint16_t delay_s =
            *(uint16_t *)message->attribute.data.value;
        ESP_RETURN_ON_FALSE(
            radar_settings_set_presence_delay_s(delay_s),
            ESP_ERR_INVALID_ARG, TAG,
            "Any Presence Delay %u s is outside %u-%u s", delay_s,
            S1_PRO_RADAR_MIN_PRESENCE_DELAY_S,
            S1_PRO_RADAR_MAX_PRESENCE_DELAY_S);
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_S16 &&
        message->attribute.data.value != NULL) {
        const int16_t coordinate_cm =
            *(int16_t *)message->attribute.data.value;
        for (uint8_t point = 0U; point < RADAR_ZONE_MAX_POINTS; ++point) {
            if (message->attribute.id ==
                S1_PRO_ATTR_EXCLUSION_POINT_X(point)) {
                ESP_RETURN_ON_FALSE(
                    radar_settings_set_exclusion_point(point, false,
                                                       coordinate_cm),
                    ESP_ERR_INVALID_ARG, TAG,
                    "Invalid Exclusion Zone P%u X coordinate %d cm",
                    point + 1U, coordinate_cm);
                return ESP_OK;
            }
            if (message->attribute.id ==
                S1_PRO_ATTR_EXCLUSION_POINT_Y(point)) {
                ESP_RETURN_ON_FALSE(
                    radar_settings_set_exclusion_point(point, true,
                                                       coordinate_cm),
                    ESP_ERR_INVALID_ARG, TAG,
                    "Invalid Exclusion Zone P%u Y coordinate %d cm",
                    point + 1U, coordinate_cm);
                return ESP_OK;
            }
        }
        for (uint8_t zone = 0U; zone < RADAR_ZONE_COUNT; ++zone) {
            for (uint8_t point = 0U; point < RADAR_ZONE_MAX_POINTS;
                 ++point) {
                if (message->attribute.id ==
                    S1_PRO_ATTR_ZONE_POINT_X(zone, point)) {
                    ESP_RETURN_ON_FALSE(
                        radar_settings_set_zone_point(
                            zone, point, false, coordinate_cm),
                        ESP_ERR_INVALID_ARG, TAG,
                        "Invalid Zone %u P%u X coordinate %d cm", zone + 1U,
                        point + 1U, coordinate_cm);
                    return ESP_OK;
                }
                if (message->attribute.id ==
                    S1_PRO_ATTR_ZONE_POINT_Y(zone, point)) {
                    ESP_RETURN_ON_FALSE(
                        radar_settings_set_zone_point(
                            zone, point, true, coordinate_cm),
                        ESP_ERR_INVALID_ARG, TAG,
                        "Invalid Zone %u P%u Y coordinate %d cm", zone + 1U,
                        point + 1U, coordinate_cm);
                    return ESP_OK;
                }
            }
        }
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8 &&
        message->attribute.data.value != NULL) {
        const uint8_t point_count =
            *(uint8_t *)message->attribute.data.value;
        if (message->attribute.id == S1_PRO_ATTR_EXCLUSION_POINTS_COUNT) {
            ESP_RETURN_ON_FALSE(
                radar_settings_set_exclusion_point_count(point_count),
                ESP_ERR_INVALID_ARG, TAG,
                "Exclusion Zone Points Count %u is outside 0-%u",
                point_count, RADAR_ZONE_MAX_POINTS);
            return ESP_OK;
        }
        for (uint8_t zone = 0U; zone < RADAR_ZONE_COUNT; ++zone) {
            if (message->attribute.id ==
                S1_PRO_ATTR_ZONE_POINTS_COUNT(zone)) {
                ESP_RETURN_ON_FALSE(
                    radar_settings_set_zone_point_count(zone, point_count),
                    ESP_ERR_INVALID_ARG, TAG,
                    "Zone %u Points Count %u is outside 0-%u", zone + 1U,
                    point_count, RADAR_ZONE_MAX_POINTS);
                return ESP_OK;
            }
        }
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 &&
        message->attribute.data.value != NULL) {
        const uint16_t value = *(uint16_t *)message->attribute.data.value;
        for (uint8_t zone = 0U; zone < RADAR_ZONE_COUNT; ++zone) {
            if (message->attribute.id ==
                S1_PRO_ATTR_ZONE_MOVEMENT_THRESHOLD(zone)) {
                ESP_RETURN_ON_FALSE(
                    radar_settings_set_zone_movement_threshold_cms(zone,
                                                                    value),
                    ESP_ERR_INVALID_ARG, TAG,
                    "Zone %u Movement Threshold %u cm/s is outside %u-%u cm/s",
                    zone + 1U, value,
                    S1_PRO_RADAR_MIN_MOVEMENT_THRESHOLD_CMS,
                    S1_PRO_RADAR_MAX_MOVEMENT_THRESHOLD_CMS);
                return ESP_OK;
            }
            if (message->attribute.id ==
                S1_PRO_ATTR_ZONE_PRESENCE_DELAY(zone)) {
                ESP_RETURN_ON_FALSE(
                    radar_settings_set_zone_presence_delay_s(zone, value),
                    ESP_ERR_INVALID_ARG, TAG,
                    "Zone %u Presence Delay %u s is outside %u-%u s",
                    zone + 1U, value, S1_PRO_RADAR_MIN_PRESENCE_DELAY_S,
                    S1_PRO_RADAR_MAX_PRESENCE_DELAY_S);
                return ESP_OK;
            }
        }
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_RADAR_FLIP_Y_AXIS &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL &&
        message->attribute.data.value != NULL) {
        radar_settings_set_flip_y_axis(
            *(bool *)message->attribute.data.value);
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_RADAR_BLUETOOTH &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL &&
        message->attribute.data.value != NULL) {
        const bool enabled = *(bool *)message->attribute.data.value;
        ESP_RETURN_ON_FALSE(
            enabled == radar_settings_get_bluetooth_enabled() ||
                radar_uart_request_bluetooth(enabled),
            ESP_ERR_NO_MEM, TAG,
            "Unable to queue Radar Bluetooth command");
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_RADAR_SINGLE_TARGET &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL &&
        message->attribute.data.value != NULL) {
        const bool enabled = *(bool *)message->attribute.data.value;
        ESP_RETURN_ON_FALSE(
            enabled == radar_settings_get_single_target() ||
                radar_uart_request_single_target(enabled),
            ESP_ERR_NO_MEM, TAG,
            "Unable to queue Radar Single Target command");
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_RADAR_RESTART_MODULE &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL &&
        message->attribute.data.value != NULL &&
        *(bool *)message->attribute.data.value) {
        ESP_RETURN_ON_FALSE(radar_uart_request_restart(), ESP_ERR_NO_MEM, TAG,
                            "Unable to queue LD2450 restart");
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_RADAR_FACTORY_RESET &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL &&
        message->attribute.data.value != NULL &&
        *(bool *)message->attribute.data.value) {
        ESP_RETURN_ON_FALSE(radar_uart_request_factory_reset(), ESP_ERR_NO_MEM,
                            TAG, "Unable to queue LD2450 factory reset");
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_ESP32_RESTART_MODULE &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL &&
        message->attribute.data.value != NULL &&
        *(bool *)message->attribute.data.value) {
        ESP_RETURN_ON_FALSE(system_control_request_restart(), ESP_ERR_NO_MEM,
                            TAG, "Unable to queue ESP32 restart");
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_ESP32_FACTORY_RESET &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL &&
        message->attribute.data.value != NULL &&
        *(bool *)message->attribute.data.value) {
        ESP_RETURN_ON_FALSE(system_control_request_factory_reset(),
                            ESP_ERR_NO_MEM, TAG,
                            "Unable to queue ESP32 factory reset");
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_BME_TEMPERATURE_OFFSET &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 &&
        message->attribute.data.value != NULL) {
        const uint16_t offset_centi_c =
            *(uint16_t *)message->attribute.data.value;
        ESP_RETURN_ON_FALSE(
            bme688_sensor_set_temperature_offset_centi_c(offset_centi_c),
            ESP_ERR_INVALID_ARG, TAG,
            "BME688 Temp Offset %.2f C is outside %.2f-%.2f C",
            offset_centi_c / 100.0,
            S1_PRO_BME688_MIN_TEMPERATURE_OFFSET_CENTI_C / 100.0,
            S1_PRO_BME688_MAX_TEMPERATURE_OFFSET_CENTI_C / 100.0);
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_SCD40_CALIBRATION_REFERENCE &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 &&
        message->attribute.data.value != NULL) {
        const uint16_t reference_ppm =
            *(uint16_t *)message->attribute.data.value;
        ESP_RETURN_ON_FALSE(
            scd40_settings_set_calibration_reference_ppm(reference_ppm),
            ESP_ERR_INVALID_ARG, TAG,
            "SCD40 Calibration Reference %u ppm is outside %u-%u ppm",
            reference_ppm, S1_PRO_SCD40_MIN_CALIBRATION_REFERENCE_PPM,
            S1_PRO_SCD40_MAX_CALIBRATION_REFERENCE_PPM);
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_SCD40_FORCED_CALIBRATION &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL &&
        message->attribute.data.value != NULL &&
        *(bool *)message->attribute.data.value) {
        const uint16_t reference_ppm =
            scd40_settings_get_calibration_reference_ppm();
        ESP_RETURN_ON_FALSE(
            scd40_sensor_request_forced_calibration(reference_ppm),
            ESP_ERR_INVALID_STATE, TAG,
            "Unable to queue SCD40 forced calibration");
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_SCD40_FACTORY_RESET &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL &&
        message->attribute.data.value != NULL &&
        *(bool *)message->attribute.data.value) {
        ESP_RETURN_ON_FALSE(scd40_sensor_request_factory_reset(),
                            ESP_ERR_INVALID_STATE, TAG,
                            "Unable to queue SCD40 factory reset");
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_SCD40_TEMPERATURE_OFFSET &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 &&
        message->attribute.data.value != NULL) {
        const uint16_t offset_centi_c =
            *(uint16_t *)message->attribute.data.value;
        ESP_RETURN_ON_FALSE(
            offset_centi_c <=
                S1_PRO_SCD40_MAX_TEMPERATURE_OFFSET_CENTI_C,
            ESP_ERR_INVALID_ARG, TAG,
            "SCD40 Temp Offset %.2f C is outside %.2f-%.2f C",
            offset_centi_c / 100.0,
            S1_PRO_SCD40_MIN_TEMPERATURE_OFFSET_CENTI_C / 100.0,
            S1_PRO_SCD40_MAX_TEMPERATURE_OFFSET_CENTI_C / 100.0);
        ESP_RETURN_ON_FALSE(
            scd40_sensor_request_temperature_offset(offset_centi_c),
            ESP_ERR_INVALID_STATE, TAG,
            "Unable to queue SCD40 Temp Offset update");
        return ESP_OK;
    }

    if (message->info.cluster == S1_PRO_CUSTOM_CLUSTER_ID &&
        message->attribute.id == S1_PRO_ATTR_BUZZER_POWER &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL &&
        message->attribute.data.value != NULL) {
        buzzer_set_power(*(bool *)message->attribute.data.value);
        return ESP_OK;
    }

    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL &&
        message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8 &&
        message->attribute.data.value != NULL) {
        rgb_led_set_level(*(uint8_t *)message->attribute.data.value);
        return ESP_OK;
    }

    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL &&
        (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID ||
         message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID) &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 &&
        message->attribute.data.value != NULL) {
        uint16_t current_x = 0;
        uint16_t current_y = 0;
        if (read_u16_attribute(ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
                               ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID,
                               &current_x) &&
            read_u16_attribute(ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
                               ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID,
                               &current_y)) {
            if (message->attribute.id ==
                ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID) {
                current_x = *(uint16_t *)message->attribute.data.value;
            } else {
                current_y = *(uint16_t *)message->attribute.data.value;
            }
            rgb_led_set_color_xy(current_x, current_y);
            return ESP_OK;
        }
    }
    return ESP_OK;
}

static esp_err_t zigbee_action_handler(
    esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    if (callback_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) {
        return handle_set_attribute(
            (const esp_zb_zcl_set_attr_value_message_t *)message);
    }
    return ESP_OK;
}

static void configure_reporting(uint16_t attribute_id, bool is_32_bit, uint16_t min_interval,
                                uint16_t max_interval)
{
    esp_zb_zcl_reporting_info_t info = {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .ep = S1_PRO_ENDPOINT_ID,
        .cluster_id = S1_PRO_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = attribute_id,
        .u.send_info.min_interval = min_interval,
        .u.send_info.max_interval = max_interval,
        .u.send_info.def_min_interval = min_interval,
        .u.send_info.def_max_interval = max_interval,
        .dst = {
            .short_addr = S1_PRO_COORDINATOR_SHORT_ADDRESS,
            .endpoint = S1_PRO_COORDINATOR_ENDPOINT_ID,
            .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        },
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    if (is_32_bit) {
        info.u.send_info.delta.u32 = 0;
    } else {
        info.u.send_info.delta.u16 = 0;
    }
    ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&info));
}

static void configure_all_reporting(void)
{
    for (uint8_t target = 0; target < 3; ++target) {
        configure_reporting(S1_PRO_ATTR_TARGET_X(target), false, 1, 1);
        configure_reporting(S1_PRO_ATTR_TARGET_Y(target), false, 1, 1);
        configure_reporting(S1_PRO_ATTR_TARGET_DISTANCE(target), false, 1, 1);
        configure_reporting(S1_PRO_ATTR_TARGET_SPEED(target), false, 1, 1);
    }
    configure_reporting(S1_PRO_ATTR_ALL_TARGETS_COUNT, false, 1, 1);
    configure_reporting(S1_PRO_ATTR_ANY_PRESENCE, false, 1, 1);
    configure_reporting(S1_PRO_ATTR_DETECTION_RANGE, false, 1, 1);
    configure_reporting(S1_PRO_ATTR_ANY_MOVEMENT, false, 1, 1);
    configure_reporting(S1_PRO_ATTR_ANY_MOVEMENT_THRESHOLD, false, 1, 1);
    configure_reporting(S1_PRO_ATTR_ANY_PRESENCE_DELAY, false, 1, 1);
    configure_reporting(S1_PRO_ATTR_RADAR_FLIP_Y_AXIS, false, 1, 1);
    configure_reporting(S1_PRO_ATTR_RADAR_BLUETOOTH, false, 1, 1);
    configure_reporting(S1_PRO_ATTR_RADAR_SINGLE_TARGET, false, 1, 1);
    configure_reporting(S1_PRO_ATTR_BME_TEMPERATURE, false, 3, 30);
    configure_reporting(S1_PRO_ATTR_BME_HUMIDITY, false, 3, 30);
    configure_reporting(S1_PRO_ATTR_BME_PRESSURE, true, 3, 30);
    configure_reporting(S1_PRO_ATTR_BME_GAS_RESISTANCE, true, 3, 30);
    configure_reporting(S1_PRO_ATTR_BME_CO2_EQUIVALENT, false, 3, 30);
    configure_reporting(S1_PRO_ATTR_BME_VOC_EQUIVALENT, false, 3, 30);
    configure_reporting(S1_PRO_ATTR_BME_BSEC_ACCURACY, false, 3, 30);
    configure_reporting(S1_PRO_ATTR_BME_IAQ, false, 3, 30);
    configure_reporting(S1_PRO_ATTR_BME_IAQ_CLASSIFICATION, false, 3, 30);
    configure_reporting(S1_PRO_ATTR_BME_TEMPERATURE_OFFSET, false, 3, 30);
    configure_reporting(S1_PRO_ATTR_SCD40_CO2, false, 5, 30);
    configure_reporting(S1_PRO_ATTR_SCD40_TEMPERATURE, false, 5, 30);
    configure_reporting(S1_PRO_ATTR_SCD40_HUMIDITY, false, 5, 30);
    configure_reporting(S1_PRO_ATTR_SCD40_CALIBRATION_REFERENCE, false, 5, 30);
    configure_reporting(S1_PRO_ATTR_SCD40_TEMPERATURE_OFFSET, false, 5, 30);
    configure_reporting(S1_PRO_ATTR_LTR390_AMBIENT_LIGHT, true, 3, 30);
    configure_reporting(S1_PRO_ATTR_LTR390_UV_INDEX, false, 3, 30);
    configure_reporting(S1_PRO_ATTR_BUZZER_POWER, false, 1, 1);
    configure_reporting(S1_PRO_ATTR_ESP32_TEMPERATURE, false, 10, 60);
    configure_reporting(S1_PRO_ATTR_ESP32_CONNECTED, false, 10, 60);
}

static bool set_attribute(uint16_t id, void *value)
{
    return esp_zb_zcl_set_attribute_val(S1_PRO_ENDPOINT_ID, S1_PRO_CUSTOM_CLUSTER_ID,
                                        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, id, value, false) ==
           ESP_ZB_ZCL_STATUS_SUCCESS;
}

static bool report_attribute(uint16_t id)
{
    esp_zb_zcl_report_attr_cmd_t command = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = S1_PRO_COORDINATOR_SHORT_ADDRESS,
            .dst_endpoint = S1_PRO_COORDINATOR_ENDPOINT_ID,
            .src_endpoint = S1_PRO_ENDPOINT_ID,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = S1_PRO_CUSTOM_CLUSTER_ID,
        .manuf_specific = 0,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .dis_defalut_resp = 1,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .attributeID = id,
    };
    return esp_zb_zcl_report_attr_cmd_req(&command) == ESP_OK;
}

static void queue_unique_report(uint16_t *queue, size_t *count, size_t capacity,
                                uint16_t id)
{
    for (size_t index = 0; index < *count; ++index) {
        if (queue[index] == id) {
            return;
        }
    }
    if (*count < capacity) {
        queue[(*count)++] = id;
    }
}

static void queue_radar_report(uint16_t id)
{
    queue_unique_report(
        s_pending_radar_reports, &s_pending_radar_report_count,
        sizeof(s_pending_radar_reports) / sizeof(s_pending_radar_reports[0]), id);
}

static void queue_bme_report(uint16_t id)
{
    queue_unique_report(
        s_pending_bme_reports, &s_pending_bme_report_count,
        sizeof(s_pending_bme_reports) / sizeof(s_pending_bme_reports[0]), id);
}

static void queue_scd40_report(uint16_t id)
{
    queue_unique_report(
        s_pending_scd40_reports, &s_pending_scd40_report_count,
        sizeof(s_pending_scd40_reports) / sizeof(s_pending_scd40_reports[0]),
        id);
}

static void queue_ltr390_report(uint16_t id)
{
    queue_unique_report(
        s_pending_ltr390_reports, &s_pending_ltr390_report_count,
        sizeof(s_pending_ltr390_reports) / sizeof(s_pending_ltr390_reports[0]),
        id);
}

static void queue_esp32_report(uint16_t id)
{
    queue_unique_report(
        s_pending_esp32_reports, &s_pending_esp32_report_count,
        sizeof(s_pending_esp32_reports) / sizeof(s_pending_esp32_reports[0]),
        id);
}

static bool flush_pending_reports(uint16_t *queue, size_t *count,
                                  size_t maximum_reports)
{
    bool success = true;
    size_t sent = 0;
    while (s_joined && sent < maximum_reports && *count > 0) {
        if (!report_attribute(queue[0])) {
            success = false;
            break;
        }
        --(*count);
        if (*count > 0) {
            memmove(&queue[0], &queue[1], *count * sizeof(queue[0]));
        }
        ++sent;
    }
    return success;
}

static bool publish_zones_configuration(const radar_zones_config_t *config)
{
    if (s_zones_config_published &&
        memcmp(config, &s_last_zones_config, sizeof(*config)) == 0) {
        return true;
    }

    bool success = true;
    for (uint8_t point = 0U; point < RADAR_ZONE_MAX_POINTS; ++point) {
        success &= set_attribute(S1_PRO_ATTR_EXCLUSION_POINT_X(point),
                                 (void *)&config->exclusion_points[point].x_cm);
        success &= set_attribute(S1_PRO_ATTR_EXCLUSION_POINT_Y(point),
                                 (void *)&config->exclusion_points[point].y_cm);
    }
    success &= set_attribute(S1_PRO_ATTR_EXCLUSION_POINTS_COUNT,
                             (void *)&config->exclusion_point_count);

    for (uint8_t zone = 0U; zone < RADAR_ZONE_COUNT; ++zone) {
        for (uint8_t point = 0U; point < RADAR_ZONE_MAX_POINTS; ++point) {
            success &= set_attribute(
                S1_PRO_ATTR_ZONE_POINT_X(zone, point),
                (void *)&config->zones[zone].points[point].x_cm);
            success &= set_attribute(
                S1_PRO_ATTR_ZONE_POINT_Y(zone, point),
                (void *)&config->zones[zone].points[point].y_cm);
        }

        success &= set_attribute(
            S1_PRO_ATTR_ZONE_POINTS_COUNT(zone),
            (void *)&config->zones[zone].point_count);
        success &= set_attribute(
            S1_PRO_ATTR_ZONE_MOVEMENT_THRESHOLD(zone),
            (void *)&config->zones[zone].movement_threshold_cms);
        success &= set_attribute(
            S1_PRO_ATTR_ZONE_PRESENCE_DELAY(zone),
            (void *)&config->zones[zone].presence_delay_s);
    }

    s_last_zones_config = *config;
    s_zones_config_published = true;
    return success;
}

static void publish_snapshot(const radar_snapshot_t *snapshot, bool fresh,
                             const bme688_snapshot_t *bme, bool bme_fresh,
                             const scd40_snapshot_t *scd40, bool scd40_fresh,
                             const ltr390_snapshot_t *ltr390, bool ltr390_fresh,
                             uint16_t detection_range_cm,
                             uint16_t movement_threshold_cms,
                             uint16_t presence_delay_s, bool flip_y_axis,
                             bool bluetooth_enabled, bool single_target,
                             const radar_zones_config_t *zones_config,
                             bool esp32_temperature_valid,
                             int16_t esp32_temperature_centi_c,
                             int64_t now_us)
{
    bool success = true;

    success &= publish_zones_configuration(zones_config);

    success &= set_attribute(S1_PRO_ATTR_DETECTION_RANGE,
                             &detection_range_cm);
    if (!s_detection_range_published ||
        detection_range_cm != s_last_detection_range_cm) {
        queue_radar_report(S1_PRO_ATTR_DETECTION_RANGE);
    }
    success &= set_attribute(S1_PRO_ATTR_ANY_MOVEMENT_THRESHOLD,
                             &movement_threshold_cms);
    if (!s_movement_threshold_published ||
        movement_threshold_cms != s_last_movement_threshold_cms) {
        queue_radar_report(S1_PRO_ATTR_ANY_MOVEMENT_THRESHOLD);
    }
    success &= set_attribute(S1_PRO_ATTR_ANY_PRESENCE_DELAY,
                             &presence_delay_s);
    if (!s_presence_delay_published ||
        presence_delay_s != s_last_presence_delay_s) {
        queue_radar_report(S1_PRO_ATTR_ANY_PRESENCE_DELAY);
    }
    success &= set_attribute(S1_PRO_ATTR_RADAR_FLIP_Y_AXIS, &flip_y_axis);
    success &=
        set_attribute(S1_PRO_ATTR_RADAR_BLUETOOTH, &bluetooth_enabled);
    success &= set_attribute(S1_PRO_ATTR_RADAR_SINGLE_TARGET, &single_target);
    if (!s_radar_controls_published || flip_y_axis != s_last_flip_y_axis) {
        queue_radar_report(S1_PRO_ATTR_RADAR_FLIP_Y_AXIS);
    }
    if (!s_radar_controls_published ||
        bluetooth_enabled != s_last_bluetooth_enabled) {
        queue_radar_report(S1_PRO_ATTR_RADAR_BLUETOOTH);
    }
    if (!s_radar_controls_published ||
        single_target != s_last_single_target) {
        queue_radar_report(S1_PRO_ATTR_RADAR_SINGLE_TARGET);
    }

    uint8_t target_count = 0;
    bool any_movement = false;
    radar_zone_target_t zone_targets[LD2450_TARGET_COUNT] = {0};
    for (uint8_t target = 0; target < 3; ++target) {
        bool present = fresh && ld2450_target_within_range(
                                    &snapshot->frame.targets[target],
                                    detection_range_cm);
        ld2450_target_t published_target = {0};
        if (present) {
            published_target = snapshot->frame.targets[target];
            published_target.x_mm = radar_transform_x_mm(
                published_target.x_mm, flip_y_axis);
            if (radar_target_is_excluded(zones_config,
                                         published_target.x_mm,
                                         published_target.y_mm)) {
                present = false;
                published_target = (ld2450_target_t){0};
            }
        }
        zone_targets[target] = (radar_zone_target_t){
            .present = present,
            .x_mm = published_target.x_mm,
            .y_mm = published_target.y_mm,
            .speed_cms = published_target.speed_cms,
        };
        if (present) {
            ++target_count;
        }
        if (ld2450_target_exceeds_speed_threshold(
                &published_target, movement_threshold_cms)) {
            any_movement = true;
        }
        success &= set_attribute(S1_PRO_ATTR_TARGET_X(target),
                                 &published_target.x_mm);
        success &= set_attribute(S1_PRO_ATTR_TARGET_Y(target),
                                 &published_target.y_mm);
        success &= set_attribute(S1_PRO_ATTR_TARGET_DISTANCE(target),
                                 &published_target.distance_mm);
        success &= set_attribute(S1_PRO_ATTR_TARGET_SPEED(target),
                                 &published_target.speed_cms);
        if (!s_radar_state_published ||
            present != s_last_targets[target].present ||
            published_target.x_mm != s_last_targets[target].x_mm) {
            queue_radar_report(S1_PRO_ATTR_TARGET_X(target));
        }
        if (!s_radar_state_published ||
            present != s_last_targets[target].present ||
            published_target.y_mm != s_last_targets[target].y_mm) {
            queue_radar_report(S1_PRO_ATTR_TARGET_Y(target));
        }
        if (!s_radar_state_published ||
            present != s_last_targets[target].present ||
            published_target.distance_mm != s_last_targets[target].distance_mm) {
            queue_radar_report(S1_PRO_ATTR_TARGET_DISTANCE(target));
        }
        if (!s_radar_state_published ||
            present != s_last_targets[target].present ||
            published_target.speed_cms != s_last_targets[target].speed_cms) {
            queue_radar_report(S1_PRO_ATTR_TARGET_SPEED(target));
        }
        s_last_targets[target] = published_target;
    }

    radar_zone_result_t zone_results[RADAR_ZONE_COUNT];
    radar_zones_evaluate(zones_config, zone_targets, LD2450_TARGET_COUNT,
                         zone_results);
    for (uint8_t zone = 0U; zone < RADAR_ZONE_COUNT; ++zone) {
        const uint16_t effective_delay_s =
            zones_config->zones[zone].point_count >= 3U
                ? zones_config->zones[zone].presence_delay_s
                : 0U;
        zone_results[zone].presence = presence_delay_update(
            &s_zone_presence_delay_states[zone], zone_results[zone].presence,
            effective_delay_s, now_us);
        success &= set_attribute(S1_PRO_ATTR_ZONE_PRESENCE(zone),
                                 &zone_results[zone].presence);
        success &= set_attribute(S1_PRO_ATTR_ZONE_MOVEMENT(zone),
                                 &zone_results[zone].movement);
        success &= set_attribute(S1_PRO_ATTR_ZONE_TARGET_COUNT(zone),
                                 &zone_results[zone].target_count);
        if (!s_radar_state_published ||
            zone_results[zone].presence !=
                s_last_zone_results[zone].presence) {
            queue_radar_report(S1_PRO_ATTR_ZONE_PRESENCE(zone));
        }
        if (!s_radar_state_published ||
            zone_results[zone].movement !=
                s_last_zone_results[zone].movement) {
            queue_radar_report(S1_PRO_ATTR_ZONE_MOVEMENT(zone));
        }
        if (!s_radar_state_published ||
            zone_results[zone].target_count !=
                s_last_zone_results[zone].target_count) {
            queue_radar_report(S1_PRO_ATTR_ZONE_TARGET_COUNT(zone));
        }
        s_last_zone_results[zone] = zone_results[zone];
    }

    const bool observed_presence = target_count > 0U;
    bool any_presence = presence_delay_update(
        &s_presence_delay_state, observed_presence, presence_delay_s, now_us);
    success &= set_attribute(S1_PRO_ATTR_ALL_TARGETS_COUNT, &target_count);
    success &= set_attribute(S1_PRO_ATTR_ANY_PRESENCE, &any_presence);
    success &= set_attribute(S1_PRO_ATTR_ANY_MOVEMENT, &any_movement);
    if (!s_radar_state_published || target_count != s_last_target_count) {
        queue_radar_report(S1_PRO_ATTR_ALL_TARGETS_COUNT);
    }
    if (!s_radar_state_published || any_presence != s_last_any_presence) {
        queue_radar_report(S1_PRO_ATTR_ANY_PRESENCE);
    }
    if (!s_radar_state_published || any_movement != s_last_any_movement) {
        queue_radar_report(S1_PRO_ATTR_ANY_MOVEMENT);
    }
    s_last_target_count = target_count;
    s_last_any_presence = any_presence;
    s_last_any_movement = any_movement;
    s_last_detection_range_cm = detection_range_cm;
    s_detection_range_published = true;
    s_last_movement_threshold_cms = movement_threshold_cms;
    s_movement_threshold_published = true;
    s_last_presence_delay_s = presence_delay_s;
    s_presence_delay_published = true;
    s_last_flip_y_axis = flip_y_axis;
    s_last_bluetooth_enabled = bluetooth_enabled;
    s_last_single_target = single_target;
    s_radar_controls_published = true;
    s_radar_state_published = true;

    bool esp32_connected = s_joined;
    success &= set_attribute(S1_PRO_ATTR_ESP32_CONNECTED, &esp32_connected);
    if (!s_esp32_status_published ||
        esp32_connected != s_last_esp32_connected) {
        queue_esp32_report(S1_PRO_ATTR_ESP32_CONNECTED);
    }
    s_last_esp32_connected = esp32_connected;
    s_esp32_status_published = true;

    if (esp32_temperature_valid) {
        success &= set_attribute(S1_PRO_ATTR_ESP32_TEMPERATURE,
                                 &esp32_temperature_centi_c);
        if (!s_esp32_temperature_published ||
            esp32_temperature_centi_c !=
                s_last_esp32_temperature_centi_c) {
            queue_esp32_report(S1_PRO_ATTR_ESP32_TEMPERATURE);
        }
        s_last_esp32_temperature_centi_c = esp32_temperature_centi_c;
        s_esp32_temperature_published = true;
    }

    const bool bme_new_data = bme_fresh &&
                              (!s_bme_state_published ||
                               bme->measurement_count != s_last_bme_measurement_count);
    if (bme_new_data) {
        bool gas_valid = bme_fresh && bme->gas_valid;
        bool bsec_valid = bme_fresh && bme->bsec_valid;
        int16_t temperature = bme->temperature_centi_c;
        uint16_t humidity = bme->humidity_centi_percent;
        uint32_t pressure = bme->pressure_pa;
        success &= set_attribute(S1_PRO_ATTR_BME_TEMPERATURE, &temperature);
        success &= set_attribute(S1_PRO_ATTR_BME_HUMIDITY, &humidity);
        success &= set_attribute(S1_PRO_ATTR_BME_PRESSURE, &pressure);
        queue_bme_report(S1_PRO_ATTR_BME_TEMPERATURE);
        queue_bme_report(S1_PRO_ATTR_BME_HUMIDITY);
        queue_bme_report(S1_PRO_ATTR_BME_PRESSURE);

        if (gas_valid) {
            uint32_t gas_resistance = bme->gas_resistance_ohm;
            success &= set_attribute(S1_PRO_ATTR_BME_GAS_RESISTANCE,
                                     &gas_resistance);
            queue_bme_report(S1_PRO_ATTR_BME_GAS_RESISTANCE);
        }
        uint8_t bsec_accuracy = bme->bsec_accuracy;
        success &=
            set_attribute(S1_PRO_ATTR_BME_BSEC_ACCURACY, &bsec_accuracy);
        if (!s_bme_state_published ||
            bsec_accuracy != s_last_bsec_accuracy) {
            queue_bme_report(S1_PRO_ATTR_BME_BSEC_ACCURACY);
        }
        if (bsec_valid) {
            uint16_t co2_equivalent = bme->co2_equivalent_ppm;
            uint16_t voc_equivalent = bme->voc_equivalent_centi_ppm;
            uint16_t iaq = bme->iaq_index;
            uint8_t iaq_classification = bme->iaq_classification;
            success &= set_attribute(S1_PRO_ATTR_BME_CO2_EQUIVALENT,
                                     &co2_equivalent);
            success &= set_attribute(S1_PRO_ATTR_BME_VOC_EQUIVALENT,
                                     &voc_equivalent);
            success &= set_attribute(S1_PRO_ATTR_BME_IAQ, &iaq);
            success &= set_attribute(S1_PRO_ATTR_BME_IAQ_CLASSIFICATION,
                                     &iaq_classification);
            queue_bme_report(S1_PRO_ATTR_BME_CO2_EQUIVALENT);
            queue_bme_report(S1_PRO_ATTR_BME_VOC_EQUIVALENT);
            queue_bme_report(S1_PRO_ATTR_BME_IAQ);
            if (!s_bme_state_published ||
                iaq_classification != s_last_iaq_classification) {
                queue_bme_report(S1_PRO_ATTR_BME_IAQ_CLASSIFICATION);
            }
        }
        s_bme_state_published = true;
        s_last_bsec_accuracy = bme->bsec_accuracy;
        s_last_iaq_classification = bme->iaq_classification;
        s_last_bme_measurement_count = bme->measurement_count;
    }

    const bool scd40_new_data =
        scd40_fresh &&
        (!s_scd40_state_published ||
         scd40->measurement_count != s_last_scd40_measurement_count);
    if (scd40_new_data) {
        uint16_t co2 = scd40->co2_ppm;
        int16_t temperature = scd40->temperature_centi_c;
        uint16_t humidity = scd40->humidity_centi_percent;
        success &= set_attribute(S1_PRO_ATTR_SCD40_CO2, &co2);
        success &=
            set_attribute(S1_PRO_ATTR_SCD40_TEMPERATURE, &temperature);
        success &= set_attribute(S1_PRO_ATTR_SCD40_HUMIDITY, &humidity);
        queue_scd40_report(S1_PRO_ATTR_SCD40_CO2);
        queue_scd40_report(S1_PRO_ATTR_SCD40_TEMPERATURE);
        queue_scd40_report(S1_PRO_ATTR_SCD40_HUMIDITY);
        s_scd40_state_published = true;
        s_last_scd40_measurement_count = scd40->measurement_count;
    }

    if (scd40->temperature_offset_valid &&
        (!s_scd40_temperature_offset_published ||
         scd40->temperature_offset_centi_c !=
             s_last_scd40_temperature_offset_centi_c)) {
        uint16_t temperature_offset_centi_c =
            scd40->temperature_offset_centi_c;
        success &= set_attribute(S1_PRO_ATTR_SCD40_TEMPERATURE_OFFSET,
                                 &temperature_offset_centi_c);
        queue_scd40_report(S1_PRO_ATTR_SCD40_TEMPERATURE_OFFSET);
        s_last_scd40_temperature_offset_centi_c =
            temperature_offset_centi_c;
        s_scd40_temperature_offset_published = true;
    }

    const bool ltr390_new_data =
        ltr390_fresh &&
        (!s_ltr390_state_published ||
         ltr390->measurement_count != s_last_ltr390_measurement_count);
    if (ltr390_new_data) {
        uint32_t ambient_light = ltr390->ambient_light_centilux;
        uint16_t uv_index = ltr390->uv_index_centi;
        success &= set_attribute(S1_PRO_ATTR_LTR390_AMBIENT_LIGHT,
                                 &ambient_light);
        success &= set_attribute(S1_PRO_ATTR_LTR390_UV_INDEX, &uv_index);
        queue_ltr390_report(S1_PRO_ATTR_LTR390_AMBIENT_LIGHT);
        queue_ltr390_report(S1_PRO_ATTR_LTR390_UV_INDEX);
        s_ltr390_state_published = true;
        s_last_ltr390_measurement_count = ltr390->measurement_count;
    }

    if (s_joined) {
        success &= flush_pending_reports(s_pending_radar_reports,
                                         &s_pending_radar_report_count,
                                         S1_PRO_RADAR_REPORTS_PER_PUBLISH);
        success &= flush_pending_reports(s_pending_bme_reports,
                                         &s_pending_bme_report_count,
                                         S1_PRO_BME_REPORTS_PER_PUBLISH);
        success &= flush_pending_reports(s_pending_scd40_reports,
                                         &s_pending_scd40_report_count,
                                         S1_PRO_SCD40_REPORTS_PER_PUBLISH);
        success &= flush_pending_reports(s_pending_ltr390_reports,
                                         &s_pending_ltr390_report_count,
                                         S1_PRO_LTR390_REPORTS_PER_PUBLISH);
        success &= flush_pending_reports(s_pending_esp32_reports,
                                         &s_pending_esp32_report_count,
                                         S1_PRO_ESP32_REPORTS_PER_PUBLISH);
    }
    (void)success;
}

static void esp32_temperature_sensor_init(void)
{
    const temperature_sensor_config_t config =
        TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    ESP_ERROR_CHECK(
        temperature_sensor_install(&config, &s_esp32_temperature_sensor));
    ESP_ERROR_CHECK(temperature_sensor_enable(s_esp32_temperature_sensor));
}

static bool esp32_temperature_sensor_read(int16_t *temperature_centi_c)
{
    float temperature_c;
    if (temperature_sensor_get_celsius(s_esp32_temperature_sensor,
                                       &temperature_c) != ESP_OK) {
        return false;
    }

    const long rounded_centi_c = lroundf(temperature_c * 100.0f);
    if (rounded_centi_c < INT16_MIN || rounded_centi_c > INT16_MAX) {
        return false;
    }
    *temperature_centi_c = (int16_t)rounded_centi_c;
    return true;
}

static void publisher_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_esp32_temperature_read_us =
        -(int64_t)S1_PRO_ESP32_TEMPERATURE_PERIOD_MS * 1000;
    int16_t esp32_temperature_centi_c = 0;
    bool esp32_temperature_valid = false;

    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(S1_PRO_PUBLISH_PERIOD_MS));
        if (!s_zigbee_ready) {
            continue;
        }

        radar_snapshot_t snapshot;
        radar_uart_get_snapshot(&snapshot);
        bme688_snapshot_t bme;
        bme688_sensor_get_snapshot(&bme);
        scd40_snapshot_t scd40;
        scd40_sensor_get_snapshot(&scd40);
        ltr390_snapshot_t ltr390;
        ltr390_sensor_get_snapshot(&ltr390);
        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_esp32_temperature_read_us >=
            (int64_t)S1_PRO_ESP32_TEMPERATURE_PERIOD_MS * 1000) {
            esp32_temperature_valid = esp32_temperature_sensor_read(
                &esp32_temperature_centi_c);
            last_esp32_temperature_read_us = now_us;
        }
        const int64_t age_ms = snapshot.valid ? (now_us - snapshot.received_at_us) / 1000 : INT64_MAX;
        const bool fresh = snapshot.valid && age_ms <= S1_PRO_RADAR_STALE_MS;
        const int64_t bme_age_ms = bme.valid ? (now_us - bme.received_at_us) / 1000 : INT64_MAX;
        const bool bme_fresh = bme.valid && bme_age_ms <= S1_PRO_BME688_STALE_MS;
        const int64_t scd40_age_ms =
            scd40.valid ? (now_us - scd40.received_at_us) / 1000 : INT64_MAX;
        const bool scd40_fresh =
            scd40.valid && scd40_age_ms <= S1_PRO_SCD40_STALE_MS;
        const int64_t ltr390_age_ms =
            ltr390.valid ? (now_us - ltr390.received_at_us) / 1000 : INT64_MAX;
        const bool ltr390_fresh =
            ltr390.valid && ltr390_age_ms <= S1_PRO_LTR390_STALE_MS;
        const uint16_t detection_range_cm =
            radar_settings_get_detection_range_cm();
        const uint16_t movement_threshold_cms =
            radar_settings_get_movement_threshold_cms();
        const uint16_t presence_delay_s =
            radar_settings_get_presence_delay_s();
        const bool flip_y_axis = radar_settings_get_flip_y_axis();
        const bool bluetooth_enabled =
            radar_settings_get_bluetooth_enabled();
        const bool single_target = radar_settings_get_single_target();
        radar_zones_config_t zones_config;
        radar_settings_get_zones(&zones_config);

        esp_zb_lock_acquire(portMAX_DELAY);
        publish_snapshot(&snapshot, fresh, &bme, bme_fresh, &scd40,
                         scd40_fresh, &ltr390, ltr390_fresh,
                         detection_range_cm, movement_threshold_cms,
                         presence_delay_s, flip_y_axis, bluetooth_enabled,
                         single_target, &zones_config,
                         esp32_temperature_valid,
                         esp32_temperature_centi_c, now_us);
        esp_zb_lock_release();

    }
}

static void zigbee_main_task(void *argument)
{
    (void)argument;
    esp_zb_cfg_t config = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg.zed_cfg = {
            .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
            .keep_alive = 3000,
        },
    };

    esp_zb_init(&config);
    esp_zb_set_rx_on_when_idle(true);
    esp_zb_ep_list_t *endpoint = create_endpoint();
    ESP_ERROR_CHECK(esp_zb_device_register(endpoint));
    esp_zb_core_action_handler_register(zigbee_action_handler);
    configure_all_reporting();
    ESP_ERROR_CHECK(esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK));
    ESP_ERROR_CHECK(esp_zb_start(false));
    s_zigbee_ready = true;
    esp_zb_stack_main_loop();
}

void zigbee_device_start(void)
{
    presence_delay_init(&s_presence_delay_state);
    for (uint8_t zone = 0U; zone < RADAR_ZONE_COUNT; ++zone) {
        presence_delay_init(&s_zone_presence_delay_states[zone]);
    }
    esp32_temperature_sensor_init();
    BaseType_t created = xTaskCreate(zigbee_main_task, "zigbee_main", 6144, NULL, 5, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    created = xTaskCreate(publisher_task, "zb_publisher", 4096, NULL, 4, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

bool zigbee_device_is_ready(void)
{
    return s_zigbee_ready;
}
