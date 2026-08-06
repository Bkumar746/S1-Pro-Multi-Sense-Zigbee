#include "buzzer.h"

#include "actuator_state.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "s1_pro_config.h"

static bool s_power;

#define BUZZER_LEDC_MODE LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_TIMER LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL LEDC_CHANNEL_0
#define BUZZER_LEDC_RESOLUTION LEDC_TIMER_10_BIT
#define BUZZER_LEDC_FULL_SCALE (1U << 10U)
#define BUZZER_LEDC_DUTY                                                    \
    ((BUZZER_LEDC_FULL_SCALE * S1_PRO_BUZZER_DUTY_PERCENT) / 100U)

static void apply_output(void)
{
    const uint32_t duty = s_power ? BUZZER_LEDC_DUTY : 0U;
    ESP_ERROR_CHECK(ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL));
}

void buzzer_init(void)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = BUZZER_LEDC_MODE,
        .duty_resolution = BUZZER_LEDC_RESOLUTION,
        .timer_num = BUZZER_LEDC_TIMER,
        .freq_hz = S1_PRO_BUZZER_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    actuator_state_snapshot_t restored;
    actuator_state_get(&restored);
    s_power = restored.buzzer_power;
    const ledc_channel_config_t channel_config = {
        .gpio_num = S1_PRO_BUZZER_GPIO,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel = BUZZER_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BUZZER_LEDC_TIMER,
        .duty = s_power ? BUZZER_LEDC_DUTY : 0U,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}

void buzzer_set_power(bool power)
{
    s_power = power;
    apply_output();
    actuator_state_set_buzzer_power(power);
}

bool buzzer_get_power(void)
{
    return s_power;
}
