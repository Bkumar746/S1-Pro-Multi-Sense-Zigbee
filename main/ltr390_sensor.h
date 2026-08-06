#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int64_t received_at_us;
    uint32_t measurement_count;
    uint32_t errors;
    uint32_t ambient_light_centilux;
    uint16_t uv_index_centi;
    uint8_t part_id;
    bool valid;
} ltr390_snapshot_t;

void ltr390_sensor_start(void);
void ltr390_sensor_get_snapshot(ltr390_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif
