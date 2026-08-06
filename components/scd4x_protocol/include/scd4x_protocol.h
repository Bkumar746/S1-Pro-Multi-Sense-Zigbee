#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCD4X_MEASUREMENT_RESPONSE_SIZE 9U
#define SCD4X_WORD_FRAME_SIZE 3U

typedef struct {
    uint16_t co2_ppm;
    int16_t temperature_centi_c;
    uint16_t humidity_centi_percent;
} scd4x_measurement_t;

uint8_t scd4x_crc8(const uint8_t *data, uint16_t length);
void scd4x_encode_word(uint16_t value,
                       uint8_t output[SCD4X_WORD_FRAME_SIZE]);
bool scd4x_decode_word(const uint8_t input[SCD4X_WORD_FRAME_SIZE],
                       uint16_t *value);
bool scd4x_decode_forced_recalibration_response(
    const uint8_t response[SCD4X_WORD_FRAME_SIZE], int16_t *correction_ppm);
uint16_t scd4x_temperature_offset_centi_c_to_raw(uint16_t offset_centi_c);
uint16_t scd4x_temperature_offset_raw_to_centi_c(uint16_t raw_offset);
bool scd4x_decode_measurement(
    const uint8_t response[SCD4X_MEASUREMENT_RESPONSE_SIZE],
    scd4x_measurement_t *measurement);

#ifdef __cplusplus
}
#endif
