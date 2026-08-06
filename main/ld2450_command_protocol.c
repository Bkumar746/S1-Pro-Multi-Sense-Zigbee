#include "ld2450_command_protocol.h"

#include <string.h>

static const uint8_t s_command_header[LD2450_COMMAND_HEADER_SIZE] = {
    0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t s_command_trailer[LD2450_COMMAND_TRAILER_SIZE] = {
    0x04, 0x03, 0x02, 0x01};

static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static void write_le16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}

size_t ld2450_build_command_frame(uint16_t command, const uint8_t *value,
                                  size_t value_length, uint8_t *frame,
                                  size_t frame_capacity)
{
    if (frame == NULL || value_length > LD2450_COMMAND_MAX_VALUE_SIZE ||
        (value == NULL && value_length != 0U)) {
        return 0U;
    }

    const size_t payload_length = 2U + value_length;
    const size_t frame_length = LD2450_COMMAND_HEADER_SIZE + 2U +
                                payload_length + LD2450_COMMAND_TRAILER_SIZE;
    if (frame_capacity < frame_length) {
        return 0U;
    }

    memcpy(frame, s_command_header, sizeof(s_command_header));
    write_le16(&frame[4], (uint16_t)payload_length);
    write_le16(&frame[6], command);
    if (value_length != 0U) {
        memcpy(&frame[8], value, value_length);
    }
    memcpy(&frame[8 + value_length], s_command_trailer,
           sizeof(s_command_trailer));
    return frame_length;
}

size_t ld2450_command_frame_size_from_prefix(const uint8_t *frame,
                                              size_t available_length)
{
    if (frame == NULL || available_length < 6U ||
        memcmp(frame, s_command_header, sizeof(s_command_header)) != 0) {
        return 0U;
    }

    const size_t payload_length = read_le16(&frame[4]);
    if (payload_length < 2U ||
        payload_length > 4U + LD2450_COMMAND_MAX_VALUE_SIZE) {
        return 0U;
    }
    return LD2450_COMMAND_HEADER_SIZE + 2U + payload_length +
           LD2450_COMMAND_TRAILER_SIZE;
}

ld2450_ack_result_t ld2450_parse_ack_frame(
    const uint8_t *frame, size_t frame_length, uint16_t expected_command,
    const uint8_t **return_value, size_t *return_value_length)
{
    if (return_value != NULL) {
        *return_value = NULL;
    }
    if (return_value_length != NULL) {
        *return_value_length = 0U;
    }

    const size_t expected_length =
        ld2450_command_frame_size_from_prefix(frame, frame_length);
    if (expected_length == 0U || frame_length != expected_length) {
        return LD2450_ACK_INVALID_FRAME;
    }

    const uint16_t payload_length = read_le16(&frame[4]);
    if (payload_length < 4U ||
        memcmp(&frame[frame_length - LD2450_COMMAND_TRAILER_SIZE],
               s_command_trailer, sizeof(s_command_trailer)) != 0) {
        return LD2450_ACK_INVALID_FRAME;
    }

    const uint16_t acknowledged_command = read_le16(&frame[6]);
    if (acknowledged_command != (uint16_t)(expected_command | 0x0100U)) {
        return LD2450_ACK_WRONG_COMMAND;
    }
    if (read_le16(&frame[8]) != 0U) {
        return LD2450_ACK_COMMAND_FAILED;
    }

    if (return_value != NULL) {
        *return_value = &frame[10];
    }
    if (return_value_length != NULL) {
        *return_value_length = payload_length - 4U;
    }
    return LD2450_ACK_VALID;
}
