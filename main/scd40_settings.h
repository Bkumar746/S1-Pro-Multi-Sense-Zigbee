#pragma once

#include <stdbool.h>
#include <stdint.h>

void scd40_settings_init(void);
uint16_t scd40_settings_get_calibration_reference_ppm(void);
bool scd40_settings_set_calibration_reference_ppm(uint16_t reference_ppm);
