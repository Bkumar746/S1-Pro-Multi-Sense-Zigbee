#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool published_presence;
    bool off_delay_pending;
    bool initialized;
    int64_t off_delay_started_us;
} presence_delay_state_t;

void presence_delay_init(presence_delay_state_t *state);
bool presence_delay_update(presence_delay_state_t *state,
                           bool observed_presence,
                           uint16_t delay_seconds,
                           int64_t now_us);
bool presence_delay_is_pending(const presence_delay_state_t *state);
