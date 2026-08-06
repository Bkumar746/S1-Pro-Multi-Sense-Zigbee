#include "ld2450_parser.h"

#include <string.h>

static const uint8_t s_header[] = {0xAA, 0xFF, 0x03, 0x00};

static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t isqrt32(uint32_t value)
{
    uint32_t root = 0;
    uint32_t bit = 1UL << 30;

    while (bit > value) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (value >= root + bit) {
            value -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    return root;
}

int16_t ld2450_decode_signed_magnitude(uint16_t raw)
{
    const int16_t magnitude = (int16_t)(raw & 0x7FFFU);
    return (raw & 0x8000U) != 0U ? magnitude : (int16_t)-magnitude;
}

uint16_t ld2450_distance_mm(int16_t x_mm, int16_t y_mm)
{
    const int32_t x = x_mm;
    const int32_t y = y_mm;
    const uint32_t squared = (uint32_t)(x * x) + (uint32_t)(y * y);
    uint32_t root = isqrt32(squared);

    if (squared - (root * root) > root) {
        ++root;
    }
    return root > UINT16_MAX ? UINT16_MAX : (uint16_t)root;
}

bool ld2450_target_within_range(const ld2450_target_t *target,
                                uint16_t range_cm)
{
    return target != NULL && target->present &&
           (uint32_t)target->distance_mm <= (uint32_t)range_cm * 10U;
}

bool ld2450_target_exceeds_speed_threshold(const ld2450_target_t *target,
                                           uint16_t threshold_cms)
{
    if (target == NULL || !target->present) {
        return false;
    }

    int32_t speed_cms = target->speed_cms;
    if (speed_cms < 0) {
        speed_cms = -speed_cms;
    }
    return (uint32_t)speed_cms > threshold_cms;
}

bool ld2450_decode_frame(const uint8_t frame[LD2450_FRAME_SIZE], ld2450_frame_t *result)
{
    if (frame == NULL || result == NULL || memcmp(frame, s_header, sizeof(s_header)) != 0 ||
        frame[28] != 0x55 || frame[29] != 0xCC) {
        return false;
    }

    for (size_t target_index = 0; target_index < LD2450_TARGET_COUNT; ++target_index) {
        const uint8_t *slot = &frame[4 + (target_index * 8)];
        ld2450_target_t *target = &result->targets[target_index];
        const uint8_t empty[8] = {0};

        target->present = memcmp(slot, empty, sizeof(empty)) != 0;
        target->x_mm = ld2450_decode_signed_magnitude(read_le16(&slot[0]));
        target->y_mm = ld2450_decode_signed_magnitude(read_le16(&slot[2]));
        target->speed_cms = ld2450_decode_signed_magnitude(read_le16(&slot[4]));
        target->distance_resolution = read_le16(&slot[6]);
        target->distance_mm = ld2450_distance_mm(target->x_mm, target->y_mm);
    }
    return true;
}

void ld2450_parser_init(ld2450_parser_t *parser, ld2450_frame_callback_t callback, void *context)
{
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->callback = callback;
    parser->callback_context = context;
}

static void retain_possible_header(ld2450_parser_t *parser)
{
    size_t keep_from = parser->length;

    for (size_t candidate = 1; candidate < parser->length; ++candidate) {
        const size_t suffix_length = parser->length - candidate;
        const size_t compare_length = suffix_length < sizeof(s_header) ? suffix_length : sizeof(s_header);
        if (memcmp(&parser->buffer[candidate], s_header, compare_length) == 0) {
            keep_from = candidate;
            break;
        }
    }

    if (keep_from < parser->length) {
        const size_t keep_length = parser->length - keep_from;
        parser->discarded_bytes += (uint32_t)keep_from;
        memmove(parser->buffer, &parser->buffer[keep_from], keep_length);
        parser->length = keep_length;
    } else {
        parser->discarded_bytes += (uint32_t)parser->length;
        parser->length = 0;
    }
}

static void feed_byte(ld2450_parser_t *parser, uint8_t byte)
{
    if (parser->length < sizeof(s_header)) {
        if (byte == s_header[parser->length]) {
            parser->buffer[parser->length++] = byte;
            return;
        }

        if (parser->length != 0) {
            ++parser->framing_errors;
            parser->discarded_bytes += (uint32_t)parser->length;
        }
        parser->length = 0;
        if (byte == s_header[0]) {
            parser->buffer[parser->length++] = byte;
        } else {
            ++parser->discarded_bytes;
        }
        return;
    }

    parser->buffer[parser->length++] = byte;
    if (parser->length != LD2450_FRAME_SIZE) {
        return;
    }

    ld2450_frame_t decoded = {0};
    if (ld2450_decode_frame(parser->buffer, &decoded)) {
        ++parser->valid_frames;
        parser->length = 0;
        if (parser->callback != NULL) {
            parser->callback(&decoded, parser->callback_context);
        }
    } else {
        ++parser->framing_errors;
        retain_possible_header(parser);
    }
}

void ld2450_parser_feed(ld2450_parser_t *parser, const uint8_t *data, size_t length)
{
    if (parser == NULL || (data == NULL && length != 0)) {
        return;
    }
    for (size_t index = 0; index < length; ++index) {
        feed_byte(parser, data[index]);
    }
}
