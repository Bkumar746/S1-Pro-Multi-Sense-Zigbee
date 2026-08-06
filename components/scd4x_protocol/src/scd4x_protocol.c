#include "scd4x_protocol.h"

#include <stddef.h>

#define SCD4X_CRC_POLYNOMIAL 0x31U
#define SCD4X_CRC_INITIAL_VALUE 0xFFU

static uint16_t read_be16(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8U) | data[1];
}

uint8_t scd4x_crc8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = SCD4X_CRC_INITIAL_VALUE;
    if (data == NULL) {
        return crc;
    }

    for (uint16_t byte = 0; byte < length; ++byte) {
        crc ^= data[byte];
        for (uint8_t bit = 0; bit < 8U; ++bit) {
            crc = (crc & 0x80U) != 0U
                      ? (uint8_t)((crc << 1U) ^ SCD4X_CRC_POLYNOMIAL)
                      : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

void scd4x_encode_word(uint16_t value,
                       uint8_t output[SCD4X_WORD_FRAME_SIZE])
{
    if (output == NULL) {
        return;
    }
    output[0] = (uint8_t)(value >> 8U);
    output[1] = (uint8_t)(value & 0xFFU);
    output[2] = scd4x_crc8(output, 2U);
}

bool scd4x_decode_word(const uint8_t input[SCD4X_WORD_FRAME_SIZE],
                       uint16_t *value)
{
    if (input == NULL || value == NULL ||
        scd4x_crc8(input, 2U) != input[2]) {
        return false;
    }
    *value = read_be16(input);
    return true;
}

bool scd4x_decode_forced_recalibration_response(
    const uint8_t response[SCD4X_WORD_FRAME_SIZE], int16_t *correction_ppm)
{
    uint16_t raw_correction = 0U;
    if (correction_ppm == NULL ||
        !scd4x_decode_word(response, &raw_correction) ||
        raw_correction == UINT16_MAX) {
        return false;
    }
    *correction_ppm = (int16_t)((int32_t)raw_correction - 0x8000L);
    return true;
}

uint16_t scd4x_temperature_offset_centi_c_to_raw(uint16_t offset_centi_c)
{
    return (uint16_t)(((uint32_t)offset_centi_c * UINT16_MAX + 8750U) /
                      17500U);
}

uint16_t scd4x_temperature_offset_raw_to_centi_c(uint16_t raw_offset)
{
    return (uint16_t)(((uint32_t)raw_offset * 17500U + 32767U) /
                      UINT16_MAX);
}

bool scd4x_decode_measurement(
    const uint8_t response[SCD4X_MEASUREMENT_RESPONSE_SIZE],
    scd4x_measurement_t *measurement)
{
    if (response == NULL || measurement == NULL) {
        return false;
    }
    for (size_t word = 0; word < 3U; ++word) {
        const size_t offset = word * 3U;
        if (scd4x_crc8(&response[offset], 2U) != response[offset + 2U]) {
            return false;
        }
    }

    const uint16_t raw_temperature = read_be16(&response[3]);
    const uint16_t raw_humidity = read_be16(&response[6]);
    measurement->co2_ppm = read_be16(&response[0]);
    measurement->temperature_centi_c =
        (int16_t)(-4500L +
                  ((17500L * raw_temperature + 32767L) / 65535L));
    measurement->humidity_centi_percent =
        (uint16_t)((10000UL * raw_humidity + 32767UL) / 65535UL);
    return true;
}
