#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LTR390_DATA_SIZE 3U

uint32_t ltr390_decode_20bit(const uint8_t data[LTR390_DATA_SIZE]);

uint32_t ltr390_als_counts_to_centilux(uint32_t counts);

uint16_t ltr390_uv_counts_to_centi_index(uint32_t counts);

#ifdef __cplusplus
}
#endif
