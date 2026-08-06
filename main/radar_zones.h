#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RADAR_ZONE_COUNT 3U
#define RADAR_ZONE_MAX_POINTS 8U
#define RADAR_ZONE_MIN_COORDINATE_CM (-600)
#define RADAR_ZONE_MAX_COORDINATE_CM 600

typedef struct {
    int16_t x_cm;
    int16_t y_cm;
} radar_zone_point_t;

typedef struct {
    radar_zone_point_t points[RADAR_ZONE_MAX_POINTS];
    uint8_t point_count;
    uint16_t movement_threshold_cms;
    uint16_t presence_delay_s;
} radar_detection_zone_config_t;

typedef struct {
    radar_zone_point_t exclusion_points[RADAR_ZONE_MAX_POINTS];
    radar_detection_zone_config_t zones[RADAR_ZONE_COUNT];
    uint8_t exclusion_point_count;
} radar_zones_config_t;

typedef struct {
    bool present;
    int16_t x_mm;
    int16_t y_mm;
    int16_t speed_cms;
} radar_zone_target_t;

typedef struct {
    uint8_t target_count;
    bool presence;
    bool movement;
} radar_zone_result_t;

bool radar_polygon_contains_mm(const radar_zone_point_t *points,
                               size_t point_count, int16_t x_mm,
                               int16_t y_mm);
bool radar_target_is_excluded(const radar_zones_config_t *config,
                              int16_t x_mm, int16_t y_mm);
void radar_zones_evaluate(const radar_zones_config_t *config,
                          const radar_zone_target_t *targets,
                          size_t target_count,
                          radar_zone_result_t results[RADAR_ZONE_COUNT]);
