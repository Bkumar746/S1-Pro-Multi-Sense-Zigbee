#include "ltr390_protocol.h"

#include <limits.h>
#include <stddef.h>

#define LTR390_20BIT_MASK 0x000FFFFFUL
#define LTR390_ALS_CENTILUX_PER_COUNT 20UL
#define LTR390_UV_COUNTS_PER_INDEX 1400UL

uint32_t ltr390_decode_20bit(const uint8_t data[LTR390_DATA_SIZE])
{
    if (data == NULL) {
        return 0U;
    }
    return (((uint32_t)data[2] << 16U) | ((uint32_t)data[1] << 8U) |
            data[0]) &
           LTR390_20BIT_MASK;
}

uint32_t ltr390_als_counts_to_centilux(uint32_t counts)
{
    counts &= LTR390_20BIT_MASK;
    return counts * LTR390_ALS_CENTILUX_PER_COUNT;
}

uint16_t ltr390_uv_counts_to_centi_index(uint32_t counts)
{
    counts &= LTR390_20BIT_MASK;
    const uint32_t centi_index =
        (counts * 100UL + LTR390_UV_COUNTS_PER_INDEX / 2UL) /
        LTR390_UV_COUNTS_PER_INDEX;
    return centi_index > UINT16_MAX ? UINT16_MAX : (uint16_t)centi_index;
}
