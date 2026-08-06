#pragma once

#include <stdbool.h>
#include <stdint.h>

void rgb_led_init(void);
void rgb_led_set_power(bool power);
void rgb_led_set_level(uint8_t level);
void rgb_led_set_color_xy(uint16_t current_x, uint16_t current_y);
void rgb_led_set_zigbee_connected(bool connected);
