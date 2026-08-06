#include "presence_delay.h"

#include <stddef.h>

void presence_delay_init(presence_delay_state_t *state)
{
    if (state == NULL) {
        return;
    }

    *state = (presence_delay_state_t){0};
}

bool presence_delay_update(presence_delay_state_t *state,
                           bool observed_presence,
                           uint16_t delay_seconds,
                           int64_t now_us)
{
    if (state == NULL) {
        return observed_presence;
    }

    if (!state->initialized) {
        state->published_presence = observed_presence;
        state->initialized = true;
        return state->published_presence;
    }

    if (observed_presence) {
        state->published_presence = true;
        state->off_delay_pending = false;
        return true;
    }

    if (!state->published_presence) {
        state->off_delay_pending = false;
        return false;
    }

    if (delay_seconds == 0U) {
        state->published_presence = false;
        state->off_delay_pending = false;
        return false;
    }

    if (!state->off_delay_pending) {
        state->off_delay_pending = true;
        state->off_delay_started_us = now_us;
        return true;
    }

    const int64_t elapsed_us = now_us >= state->off_delay_started_us
                                   ? now_us - state->off_delay_started_us
                                   : 0;
    const int64_t delay_us = (int64_t)delay_seconds * 1000000LL;
    if (elapsed_us >= delay_us) {
        state->published_presence = false;
        state->off_delay_pending = false;
    }

    return state->published_presence;
}

bool presence_delay_is_pending(const presence_delay_state_t *state)
{
    return state != NULL && state->off_delay_pending;
}
