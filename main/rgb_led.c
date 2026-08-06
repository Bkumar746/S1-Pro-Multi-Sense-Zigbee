#include "rgb_led.h"

#include <math.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdint.h>

#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "actuator_state.h"
#include "s1_pro_config.h"

static led_strip_handle_t s_strip;
static SemaphoreHandle_t s_mutex;
static bool s_power;
static atomic_bool s_zigbee_connected;
static bool s_applied_zigbee_connected;
static uint8_t s_level = S1_PRO_RGB_LED_DEFAULT_LEVEL;
static uint8_t s_red = UINT8_MAX;
static uint8_t s_green = UINT8_MAX;
static uint8_t s_blue = UINT8_MAX;
static uint16_t s_current_x = S1_PRO_RGB_LED_DEFAULT_X;
static uint16_t s_current_y = S1_PRO_RGB_LED_DEFAULT_Y;
static float s_breath_phase;

static float clamp_unit(float value)
{
    if (value < 0.0F) {
        return 0.0F;
    }
    if (value > 1.0F) {
        return 1.0F;
    }
    return value;
}

static float linear_to_srgb(float value)
{
    value = clamp_unit(value);
    if (value <= 0.0031308F) {
        return 12.92F * value;
    }
    return 1.055F * powf(value, 1.0F / 2.4F) - 0.055F;
}

static void apply_output(void)
{
    if (!s_power) {
        ESP_ERROR_CHECK(led_strip_clear(s_strip));
        return;
    }

    const float brightness = (float)s_level / 254.0F;
    const uint8_t red = (uint8_t)lroundf((float)s_red * brightness);
    const uint8_t green = (uint8_t)lroundf((float)s_green * brightness);
    const uint8_t blue = (uint8_t)lroundf((float)s_blue * brightness);
    ESP_ERROR_CHECK(led_strip_set_pixel(s_strip, 0, red, green, blue));
    ESP_ERROR_CHECK(led_strip_refresh(s_strip));
}

static void apply_breathing_output(void)
{
    s_breath_phase += S1_PRO_RGB_LED_BREATH_PHASE_STEP;
    if (s_breath_phase > 1.0F) {
        s_breath_phase -= 1.0F;
    }

    const float wave =
        (sinf(s_breath_phase * 2.0F * (float)M_PI) + 1.0F) * 0.5F;
    const float brightness =
        wave * S1_PRO_RGB_LED_BREATH_RANGE +
        S1_PRO_RGB_LED_BREATH_MIN_BRIGHTNESS;
    const uint8_t red =
        (uint8_t)(brightness * S1_PRO_RGB_LED_BREATH_RED);
    const uint8_t blue =
        (uint8_t)(brightness * S1_PRO_RGB_LED_BREATH_BLUE);
    ESP_ERROR_CHECK(led_strip_set_pixel(s_strip, 0, red, 0, blue));
    ESP_ERROR_CHECK(led_strip_refresh(s_strip));
}

static void breathing_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(S1_PRO_RGB_LED_BREATH_INTERVAL_MS));
        const bool connected = atomic_load(&s_zigbee_connected);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        if (connected != s_applied_zigbee_connected) {
            s_applied_zigbee_connected = connected;
            if (connected) {
                apply_output();
            } else {
                s_breath_phase = 0.0F;
            }
        }
        if (!connected) {
            apply_breathing_output();
        }
        xSemaphoreGive(s_mutex);
    }
}

void rgb_led_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    const led_strip_config_t strip_config = {
        .strip_gpio_num = S1_PRO_RGB_LED_GPIO,
        .max_leds = S1_PRO_RGB_LED_COUNT,
    };
    const led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10U * 1000U * 1000U,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip));
    actuator_state_snapshot_t restored;
    actuator_state_get(&restored);
    s_power = restored.led_power;
    s_level = restored.led_level;
    rgb_led_set_color_xy(restored.led_x, restored.led_y);

    BaseType_t created =
        xTaskCreate(breathing_task, "led_breath", 3072, NULL, 2, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

void rgb_led_set_power(bool power)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_power = power;
    if (atomic_load(&s_zigbee_connected)) {
        apply_output();
    }
    xSemaphoreGive(s_mutex);
    actuator_state_set_led_power(power);
}

void rgb_led_set_level(uint8_t level)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_level = level == UINT8_MAX ? 254U : level;
    if (atomic_load(&s_zigbee_connected)) {
        apply_output();
    }
    const uint8_t stored_level = s_level;
    xSemaphoreGive(s_mutex);
    actuator_state_set_led_level(stored_level);
}

void rgb_led_set_color_xy(uint16_t current_x, uint16_t current_y)
{
    const float x = (float)current_x / 65535.0F;
    const float y = (float)current_y / 65535.0F;
    if (y <= 0.0001F) {
        return;
    }

    const float capital_x = x / y;
    const float capital_y = 1.0F;
    const float capital_z = (1.0F - x - y) / y;
    float red = 3.2406F * capital_x - 1.5372F * capital_y - 0.4986F * capital_z;
    float green = -0.9689F * capital_x + 1.8758F * capital_y + 0.0415F * capital_z;
    float blue = 0.0557F * capital_x - 0.2040F * capital_y + 1.0570F * capital_z;

    red = red < 0.0F ? 0.0F : red;
    green = green < 0.0F ? 0.0F : green;
    blue = blue < 0.0F ? 0.0F : blue;
    const float maximum = fmaxf(red, fmaxf(green, blue));
    if (maximum > 1.0F) {
        red /= maximum;
        green /= maximum;
        blue /= maximum;
    }

    const uint8_t converted_red =
        (uint8_t)lroundf(linear_to_srgb(red) * 255.0F);
    const uint8_t converted_green =
        (uint8_t)lroundf(linear_to_srgb(green) * 255.0F);
    const uint8_t converted_blue =
        (uint8_t)lroundf(linear_to_srgb(blue) * 255.0F);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_red = converted_red;
    s_green = converted_green;
    s_blue = converted_blue;
    s_current_x = current_x;
    s_current_y = current_y;
    if (atomic_load(&s_zigbee_connected)) {
        apply_output();
    }
    xSemaphoreGive(s_mutex);
    actuator_state_set_led_color(current_x, current_y);
}

void rgb_led_set_zigbee_connected(bool connected)
{
    atomic_store(&s_zigbee_connected, connected);
}
