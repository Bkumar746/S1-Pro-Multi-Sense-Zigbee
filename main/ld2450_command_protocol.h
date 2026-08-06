#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LD2450_COMMAND_HEADER_SIZE 4U
#define LD2450_COMMAND_TRAILER_SIZE 4U
#define LD2450_COMMAND_MAX_VALUE_SIZE 64U
#define LD2450_COMMAND_MAX_FRAME_SIZE                                      \
    (LD2450_COMMAND_HEADER_SIZE + 2U + 2U + LD2450_COMMAND_MAX_VALUE_SIZE + \
     LD2450_COMMAND_TRAILER_SIZE)

#define LD2450_COMMAND_ENABLE_CONFIGURATION 0x00FFU
#define LD2450_COMMAND_END_CONFIGURATION 0x00FEU
#define LD2450_COMMAND_SINGLE_TARGET 0x0080U
#define LD2450_COMMAND_MULTI_TARGET 0x0090U
#define LD2450_COMMAND_QUERY_TRACKING_MODE 0x0091U
#define LD2450_COMMAND_FACTORY_RESET 0x00A2U
#define LD2450_COMMAND_RESTART 0x00A3U
#define LD2450_COMMAND_BLUETOOTH 0x00A4U

typedef enum {
    LD2450_ACK_VALID = 0,
    LD2450_ACK_INVALID_FRAME,
    LD2450_ACK_WRONG_COMMAND,
    LD2450_ACK_COMMAND_FAILED,
} ld2450_ack_result_t;

size_t ld2450_build_command_frame(uint16_t command, const uint8_t *value,
                                  size_t value_length, uint8_t *frame,
                                  size_t frame_capacity);
size_t ld2450_command_frame_size_from_prefix(const uint8_t *frame,
                                              size_t available_length);
ld2450_ack_result_t ld2450_parse_ack_frame(
    const uint8_t *frame, size_t frame_length, uint16_t expected_command,
    const uint8_t **return_value, size_t *return_value_length);
