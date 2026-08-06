#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LD2450_FRAME_SIZE 30U
#define LD2450_TARGET_COUNT 3U

typedef struct {
    int16_t x_mm;
    int16_t y_mm;
    int16_t speed_cms;
    uint16_t distance_mm;
    uint16_t distance_resolution;
    bool present;
} ld2450_target_t;

typedef struct {
    ld2450_target_t targets[LD2450_TARGET_COUNT];
} ld2450_frame_t;

typedef void (*ld2450_frame_callback_t)(const ld2450_frame_t *frame, void *context);

typedef struct {
    uint8_t buffer[LD2450_FRAME_SIZE];
    size_t length;
    uint32_t valid_frames;
    uint32_t framing_errors;
    uint32_t discarded_bytes;
    ld2450_frame_callback_t callback;
    void *callback_context;
} ld2450_parser_t;

void ld2450_parser_init(ld2450_parser_t *parser, ld2450_frame_callback_t callback, void *context);
void ld2450_parser_feed(ld2450_parser_t *parser, const uint8_t *data, size_t length);
bool ld2450_decode_frame(const uint8_t frame[LD2450_FRAME_SIZE], ld2450_frame_t *result);
int16_t ld2450_decode_signed_magnitude(uint16_t raw);
uint16_t ld2450_distance_mm(int16_t x_mm, int16_t y_mm);
bool ld2450_target_within_range(const ld2450_target_t *target,
                                uint16_t range_cm);
bool ld2450_target_exceeds_speed_threshold(const ld2450_target_t *target,
                                           uint16_t threshold_cms);

#ifdef __cplusplus
}
#endif
