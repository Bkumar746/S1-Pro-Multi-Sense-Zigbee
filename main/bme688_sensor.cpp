#include "bme688_sensor.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "bsec2.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "i2c_bus.h"
#include "s1_pro_config.h"

namespace {

constexpr char BSEC_NVS_NAMESPACE[] = "bsec2";
constexpr char BSEC_NVS_STATE_KEY[] = "state";
constexpr char BSEC_NVS_VERSION_KEY[] = "version";
constexpr char BSEC_NVS_TEMPERATURE_OFFSET_KEY[] = "temp_offset";

portMUX_TYPE s_snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
bme688_snapshot_t s_snapshot{};
i2c_master_dev_handle_t s_device;
Bsec2 s_bsec;
bool s_seen_co2;
bool s_seen_voc;
bool s_seen_iaq;
uint8_t s_iaq_accuracy;
bool s_state_available;
int64_t s_last_state_save_us;
uint16_t s_temperature_offset_centi_c =
    S1_PRO_BME688_DEFAULT_TEMPERATURE_OFFSET_CENTI_C;
uint16_t s_applied_temperature_offset_centi_c = UINT16_MAX;
bool s_temperature_offset_dirty;
int64_t s_temperature_offset_changed_at_us;

BME68X_INTF_RET_TYPE bme688_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                     uint32_t length, void *intf_ptr)
{
    const auto device = static_cast<i2c_master_dev_handle_t>(intf_ptr);
    const esp_err_t status = i2c_master_transmit_receive(
        device, &reg_addr, 1, reg_data, length, S1_PRO_I2C_TIMEOUT_MS);
    return status == ESP_OK ? BME68X_INTF_RET_SUCCESS : -1;
}

BME68X_INTF_RET_TYPE bme688_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                      uint32_t length, void *intf_ptr)
{
    const auto device = static_cast<i2c_master_dev_handle_t>(intf_ptr);
    uint8_t transaction[32];
    if (length + 1U > sizeof(transaction)) {
        return -1;
    }

    transaction[0] = reg_addr;
    std::memcpy(&transaction[1], reg_data, length);
    const esp_err_t status = i2c_master_transmit(
        device, transaction, length + 1U, S1_PRO_I2C_TIMEOUT_MS);
    return status == ESP_OK ? BME68X_INTF_RET_SUCCESS : -1;
}

void bme688_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    if (period_us >= 1000U) {
        vTaskDelay(pdMS_TO_TICKS((period_us + 999U) / 1000U));
    } else {
        esp_rom_delay_us(period_us);
    }
}

unsigned long bsec_millis()
{
    return static_cast<unsigned long>(esp_timer_get_time() / 1000ULL);
}

void record_error()
{
    portENTER_CRITICAL(&s_snapshot_lock);
    ++s_snapshot.errors;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

int16_t to_centi_signed(float value)
{
    const float scaled = value * 100.0F;
    const long rounded = std::lround(scaled);
    return static_cast<int16_t>(std::clamp<long>(rounded, INT16_MIN, INT16_MAX));
}

uint16_t to_centi_unsigned(float value)
{
    if (value <= 0.0F) {
        return 0;
    }
    const long rounded = std::lround(value * 100.0F);
    return static_cast<uint16_t>(std::clamp<long>(rounded, 0, UINT16_MAX));
}

uint16_t to_unsigned(float value)
{
    if (value <= 0.0F) {
        return 0;
    }
    const long rounded = std::lround(value);
    return static_cast<uint16_t>(std::clamp<long>(rounded, 0, UINT16_MAX));
}

uint8_t classify_iaq(uint16_t iaq)
{
    if (iaq <= 50U) {
        return 0U;
    }
    if (iaq <= 100U) {
        return 1U;
    }
    if (iaq <= 150U) {
        return 2U;
    }
    if (iaq <= 200U) {
        return 3U;
    }
    if (iaq <= 250U) {
        return 4U;
    }
    if (iaq <= 350U) {
        return 5U;
    }
    return 6U;
}

uint32_t current_bsec_version()
{
    return (static_cast<uint32_t>(s_bsec.version.major) << 24U) |
           (static_cast<uint32_t>(s_bsec.version.minor) << 16U) |
           (static_cast<uint32_t>(s_bsec.version.major_bugfix) << 8U) |
           static_cast<uint32_t>(s_bsec.version.minor_bugfix);
}

void load_bsec_state()
{
    nvs_handle_t handle = 0;
    esp_err_t status = nvs_open(BSEC_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (status == ESP_ERR_NVS_NOT_FOUND) {
        return;
    }
    if (status != ESP_OK) {
        return;
    }

    uint32_t stored_version = 0;
    size_t state_size = BSEC_MAX_STATE_BLOB_SIZE;
    uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
    status = nvs_get_u32(handle, BSEC_NVS_VERSION_KEY, &stored_version);
    if (status == ESP_OK && stored_version == current_bsec_version()) {
        status = nvs_get_blob(handle, BSEC_NVS_STATE_KEY, state, &state_size);
        if (status == ESP_OK && state_size == BSEC_MAX_STATE_BLOB_SIZE &&
            s_bsec.setState(state)) {
            s_state_available = true;
            s_last_state_save_us = esp_timer_get_time();
        }
    }
    nvs_close(handle);
}

void save_bsec_state_if_due(uint8_t accuracy)
{
    if (accuracy == 0U) {
        return;
    }
    const int64_t now_us = esp_timer_get_time();
    const int64_t save_period_us =
        static_cast<int64_t>(S1_PRO_BSEC_STATE_SAVE_PERIOD_MS) * 1000LL;
    if (s_state_available && now_us - s_last_state_save_us < save_period_us) {
        return;
    }

    uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
    if (!s_bsec.getState(state)) {
        record_error();
        return;
    }

    nvs_handle_t handle = 0;
    esp_err_t status = nvs_open(BSEC_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (status == ESP_OK) {
        status = nvs_set_u32(handle, BSEC_NVS_VERSION_KEY, current_bsec_version());
    }
    if (status == ESP_OK) {
        status = nvs_set_blob(handle, BSEC_NVS_STATE_KEY, state, sizeof(state));
    }
    if (status == ESP_OK) {
        status = nvs_commit(handle);
    }
    if (status == ESP_OK) {
        s_state_available = true;
        s_last_state_save_us = now_us;
    } else {
        record_error();
    }
    if (handle != 0) {
        nvs_close(handle);
    }
}

bool temperature_offset_is_valid(uint16_t offset_centi_c)
{
    return offset_centi_c <=
           S1_PRO_BME688_MAX_TEMPERATURE_OFFSET_CENTI_C;
}

void load_temperature_offset()
{
    nvs_handle_t handle = 0;
    const esp_err_t status =
        nvs_open(BSEC_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (status == ESP_OK) {
        uint16_t restored_offset_centi_c = 0U;
        const esp_err_t read_status = nvs_get_u16(
            handle, BSEC_NVS_TEMPERATURE_OFFSET_KEY,
            &restored_offset_centi_c);
        if (read_status == ESP_OK &&
            temperature_offset_is_valid(restored_offset_centi_c)) {
            s_temperature_offset_centi_c = restored_offset_centi_c;
        }
        nvs_close(handle);
    }
}

void apply_temperature_offset_if_changed()
{
    uint16_t offset_centi_c = 0U;
    portENTER_CRITICAL(&s_snapshot_lock);
    offset_centi_c = s_temperature_offset_centi_c;
    portEXIT_CRITICAL(&s_snapshot_lock);

    if (offset_centi_c == s_applied_temperature_offset_centi_c) {
        return;
    }
    s_bsec.setTemperatureOffset(offset_centi_c / 100.0F);
    s_applied_temperature_offset_centi_c = offset_centi_c;
}

void save_temperature_offset_if_due()
{
    const int64_t now_us = esp_timer_get_time();
    const int64_t save_delay_us =
        static_cast<int64_t>(S1_PRO_BME688_SETTINGS_SAVE_DELAY_MS) * 1000LL;
    uint16_t offset_centi_c = 0U;
    bool save_due = false;

    portENTER_CRITICAL(&s_snapshot_lock);
    if (s_temperature_offset_dirty &&
        now_us - s_temperature_offset_changed_at_us >= save_delay_us) {
        offset_centi_c = s_temperature_offset_centi_c;
        s_temperature_offset_dirty = false;
        save_due = true;
    }
    portEXIT_CRITICAL(&s_snapshot_lock);
    if (!save_due) {
        return;
    }

    nvs_handle_t handle = 0;
    esp_err_t status =
        nvs_open(BSEC_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (status == ESP_OK) {
        status = nvs_set_u16(handle, BSEC_NVS_TEMPERATURE_OFFSET_KEY,
                             offset_centi_c);
    }
    if (status == ESP_OK) {
        status = nvs_commit(handle);
    }
    if (handle != 0) {
        nvs_close(handle);
    }
    if (status == ESP_OK) {
        return;
    }

    portENTER_CRITICAL(&s_snapshot_lock);
    s_temperature_offset_dirty = true;
    s_temperature_offset_changed_at_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_snapshot_lock);
}

void store_measurement(const bme68xData data, const bsecOutputs outputs, const Bsec2 bsec)
{
    (void)bsec;
    float temperature_c = data.temperature;
    float humidity_percent = data.humidity;
    float co2_equivalent_ppm = 0.0F;
    float voc_equivalent_ppm = 0.0F;
    float iaq_index = 0.0F;
    bool co2_seen_now = false;
    bool voc_seen_now = false;
    bool iaq_seen_now = false;

    for (uint8_t index = 0; index < outputs.nOutputs; ++index) {
        const bsecData &output = outputs.output[index];
        switch (output.sensor_id) {
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
            temperature_c = output.signal;
            break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
            humidity_percent = output.signal;
            break;
        case BSEC_OUTPUT_CO2_EQUIVALENT:
            co2_equivalent_ppm = output.signal;
            co2_seen_now = true;
            s_seen_co2 = true;
            break;
        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
            voc_equivalent_ppm = output.signal;
            voc_seen_now = true;
            s_seen_voc = true;
            break;
        case BSEC_OUTPUT_IAQ:
            iaq_index = output.signal;
            s_iaq_accuracy = output.accuracy;
            iaq_seen_now = true;
            s_seen_iaq = true;
            break;
        default:
            break;
        }
    }

    const bool gas_valid = (data.status & BME68X_GASM_VALID_MSK) != 0 &&
                           (data.status & BME68X_HEAT_STAB_MSK) != 0;
    const uint8_t bsec_accuracy = s_seen_iaq ? s_iaq_accuracy : 0U;

    portENTER_CRITICAL(&s_snapshot_lock);
    s_snapshot.received_at_us = esp_timer_get_time();
    s_snapshot.temperature_centi_c = to_centi_signed(temperature_c);
    s_snapshot.humidity_centi_percent = to_centi_unsigned(humidity_percent);
    s_snapshot.pressure_pa = data.pressure <= 0.0F
                                 ? 0U
                                 : static_cast<uint32_t>(std::lround(data.pressure * 100.0F));
    if (gas_valid) {
        s_snapshot.gas_resistance_ohm = data.gas_resistance <= 0.0F
                                            ? 0U
                                            : static_cast<uint32_t>(std::lround(data.gas_resistance));
    }
    if (co2_seen_now) {
        s_snapshot.co2_equivalent_ppm = to_unsigned(co2_equivalent_ppm);
    }
    if (voc_seen_now) {
        s_snapshot.voc_equivalent_centi_ppm = to_centi_unsigned(voc_equivalent_ppm);
    }
    if (iaq_seen_now) {
        s_snapshot.iaq_index = std::min<uint16_t>(to_unsigned(iaq_index), 500U);
        s_snapshot.iaq_classification = classify_iaq(s_snapshot.iaq_index);
    }
    s_snapshot.bsec_accuracy = bsec_accuracy;
    s_snapshot.gas_valid = gas_valid;
    s_snapshot.bsec_valid = s_seen_co2 && s_seen_voc && s_seen_iaq;
    s_snapshot.valid = true;
    ++s_snapshot.measurement_count;
    const bme688_snapshot_t snapshot = s_snapshot;
    portEXIT_CRITICAL(&s_snapshot_lock);

    save_bsec_state_if_due(snapshot.bsec_accuracy);
}

bool initialize_bsec()
{
    if (!s_bsec.begin(BME68X_I2C_INTF, bme688_i2c_read, bme688_i2c_write,
                      bme688_delay_us, s_device, bsec_millis)) {
        return false;
    }

    s_applied_temperature_offset_centi_c = UINT16_MAX;
    apply_temperature_offset_if_changed();
    load_bsec_state();

    bsecSensor sensors[] = {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_STABILIZATION_STATUS,
        BSEC_OUTPUT_RUN_IN_STATUS,
    };
    const bool subscription_ok =
        s_bsec.updateSubscription(sensors, ARRAY_LEN(sensors), BSEC_SAMPLE_RATE_LP);
    if (!subscription_ok && s_bsec.status < BSEC_OK) {
        return false;
    }
    s_bsec.attachCallback(store_measurement);

    return true;
}

void bme688_task(void *argument)
{
    (void)argument;
    const esp_err_t i2c_status = s1_pro_i2c_add_device(
        S1_PRO_BME688_I2C_ADDRESS, &s_device);
    if (i2c_status != ESP_OK) {
        record_error();
        vTaskDelete(nullptr);
        return;
    }

    while (!initialize_bsec()) {
        record_error();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    while (true) {
        apply_temperature_offset_if_changed();
        save_temperature_offset_if_due();
        if (!s_bsec.run()) {
            record_error();
            vTaskDelay(pdMS_TO_TICKS(250));
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

}

extern "C" void bme688_sensor_get_snapshot(bme688_snapshot_t *snapshot)
{
    if (snapshot == nullptr) {
        return;
    }
    portENTER_CRITICAL(&s_snapshot_lock);
    *snapshot = s_snapshot;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

extern "C" void bme688_sensor_start(void)
{
    load_temperature_offset();
    const BaseType_t created = xTaskCreate(bme688_task, "bme688_bsec", 8192, nullptr, 5, nullptr);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

extern "C" uint16_t bme688_sensor_get_temperature_offset_centi_c(void)
{
    portENTER_CRITICAL(&s_snapshot_lock);
    const uint16_t offset_centi_c = s_temperature_offset_centi_c;
    portEXIT_CRITICAL(&s_snapshot_lock);
    return offset_centi_c;
}

extern "C" bool bme688_sensor_set_temperature_offset_centi_c(
    uint16_t offset_centi_c)
{
    if (!temperature_offset_is_valid(offset_centi_c)) {
        return false;
    }

    portENTER_CRITICAL(&s_snapshot_lock);
    if (s_temperature_offset_centi_c != offset_centi_c) {
        s_temperature_offset_centi_c = offset_centi_c;
        s_temperature_offset_dirty = true;
        s_temperature_offset_changed_at_us = esp_timer_get_time();
    }
    portEXIT_CRITICAL(&s_snapshot_lock);
    return true;
}
