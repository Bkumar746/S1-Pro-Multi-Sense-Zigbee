#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ld2450_parser.h"

typedef struct {
    ld2450_frame_t frame;
    int64_t received_at_us;
    uint32_t valid_frames;
    uint32_t framing_errors;
    uint32_t uart_errors;
    uint32_t command_errors;
    bool valid;
} radar_snapshot_t;

void radar_uart_start(void);
void radar_uart_get_snapshot(radar_snapshot_t *snapshot);
bool radar_uart_request_bluetooth(bool enabled);
bool radar_uart_request_single_target(bool enabled);
bool radar_uart_request_restart(void);
bool radar_uart_request_factory_reset(void);
