#include "factory_reset.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "s1_pro_config.h"
#include "zigbee_device.h"

static void factory_reset_button_task(void *argument)
{
    (void)argument;
    TickType_t pressed_at = 0;
    bool reset_triggered = false;

    while (true) {
        const bool pressed = gpio_get_level(S1_PRO_BOOT_GPIO) == 0;
        const TickType_t now = xTaskGetTickCount();
        if (pressed && pressed_at == 0) {
            pressed_at = now;
            reset_triggered = false;
        } else if (!pressed) {
            pressed_at = 0;
            reset_triggered = false;
        } else if (!reset_triggered && zigbee_device_is_ready() &&
                   (now - pressed_at) >= pdMS_TO_TICKS(S1_PRO_FACTORY_RESET_HOLD_MS)) {
            reset_triggered = true;
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_factory_reset();
            esp_zb_lock_release();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void factory_reset_button_start(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << S1_PRO_BOOT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    BaseType_t created = xTaskCreate(factory_reset_button_task, "factory_reset", 3072, NULL, 3, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
