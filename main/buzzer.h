#pragma once

#include <stdbool.h>

void buzzer_init(void);
void buzzer_set_power(bool power);
bool buzzer_get_power(void);
