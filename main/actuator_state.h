#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool led_power;
    uint8_t led_level;
    uint16_t led_x;
    uint16_t led_y;
    bool buzzer_power;
} actuator_state_snapshot_t;

void actuator_state_init(void);
void actuator_state_get(actuator_state_snapshot_t *state);
void actuator_state_set_led_power(bool power);
void actuator_state_set_led_level(uint8_t level);
void actuator_state_set_led_color(uint16_t x, uint16_t y);
void actuator_state_set_buzzer_power(bool power);
