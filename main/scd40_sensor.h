#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int64_t received_at_us;
    uint64_t serial_number;
    uint32_t measurement_count;
    uint32_t errors;
    int16_t temperature_centi_c;
    uint16_t humidity_centi_percent;
    uint16_t co2_ppm;
    uint16_t temperature_offset_centi_c;
    bool temperature_offset_valid;
    bool valid;
} scd40_snapshot_t;

void scd40_sensor_start(void);
void scd40_sensor_get_snapshot(scd40_snapshot_t *snapshot);
bool scd40_sensor_request_forced_calibration(uint16_t reference_ppm);
bool scd40_sensor_request_factory_reset(void);
bool scd40_sensor_request_temperature_offset(uint16_t offset_centi_c);

#ifdef __cplusplus
}
#endif
