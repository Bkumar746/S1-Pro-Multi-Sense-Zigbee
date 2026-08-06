#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int64_t received_at_us;
    uint32_t pressure_pa;
    uint32_t gas_resistance_ohm;
    uint32_t measurement_count;
    uint32_t errors;
    int16_t temperature_centi_c;
    uint16_t humidity_centi_percent;
    uint16_t co2_equivalent_ppm;
    uint16_t voc_equivalent_centi_ppm;
    uint16_t iaq_index;
    uint8_t bsec_accuracy;
    uint8_t iaq_classification;
    bool gas_valid;
    bool bsec_valid;
    bool valid;
} bme688_snapshot_t;

void bme688_sensor_start(void);
void bme688_sensor_get_snapshot(bme688_snapshot_t *snapshot);
uint16_t bme688_sensor_get_temperature_offset_centi_c(void);
bool bme688_sensor_set_temperature_offset_centi_c(uint16_t offset_centi_c);

#ifdef __cplusplus
}
#endif
