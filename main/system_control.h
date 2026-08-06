#pragma once

#include <stdbool.h>

void system_control_start(void);
bool system_control_request_restart(void);
bool system_control_request_factory_reset(void);
