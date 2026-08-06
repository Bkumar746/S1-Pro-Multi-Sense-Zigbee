#include "radar_zones.h"

#include <limits.h>
#include <string.h>

static int64_t point_cross_product(int32_t ax, int32_t ay, int32_t bx,
                                   int32_t by, int32_t px, int32_t py)
{
    return (int64_t)(px - ax) * (by - ay) -
           (int64_t)(py - ay) * (bx - ax);
}

static bool point_on_segment(int32_t ax, int32_t ay, int32_t bx, int32_t by,
                             int32_t px, int32_t py)
{
    if (point_cross_product(ax, ay, bx, by, px, py) != 0) {
        return false;
    }

    const int32_t min_x = ax < bx ? ax : bx;
    const int32_t max_x = ax > bx ? ax : bx;
    const int32_t min_y = ay < by ? ay : by;
    const int32_t max_y = ay > by ? ay : by;
    return px >= min_x && px <= max_x && py >= min_y && py <= max_y;
}

bool radar_polygon_contains_mm(const radar_zone_point_t *points,
                               size_t point_count, int16_t x_mm,
                               int16_t y_mm)
{
    if (points == NULL || point_count < 3U ||
        point_count > RADAR_ZONE_MAX_POINTS) {
        return false;
    }

    const int32_t px = x_mm;
    const int32_t py = y_mm;
    bool inside = false;
    size_t previous = point_count - 1U;

    for (size_t current = 0U; current < point_count; ++current) {
        const int32_t ax = (int32_t)points[previous].x_cm * 10;
        const int32_t ay = (int32_t)points[previous].y_cm * 10;
        const int32_t bx = (int32_t)points[current].x_cm * 10;
        const int32_t by = (int32_t)points[current].y_cm * 10;

        if (point_on_segment(ax, ay, bx, by, px, py)) {
            return true;
        }

        const bool crosses_y = (ay > py) != (by > py);
        if (crosses_y) {
            const int64_t left = (int64_t)(px - ax) * (by - ay);
            const int64_t right = (int64_t)(bx - ax) * (py - ay);
            if (((by > ay) && left < right) ||
                ((by < ay) && left > right)) {
                inside = !inside;
            }
        }
        previous = current;
    }

    return inside;
}

bool radar_target_is_excluded(const radar_zones_config_t *config,
                              int16_t x_mm, int16_t y_mm)
{
    if (config == NULL) {
        return false;
    }
    return radar_polygon_contains_mm(config->exclusion_points,
                                     config->exclusion_point_count, x_mm,
                                     y_mm);
}

void radar_zones_evaluate(const radar_zones_config_t *config,
                          const radar_zone_target_t *targets,
                          size_t target_count,
                          radar_zone_result_t results[RADAR_ZONE_COUNT])
{
    if (results == NULL) {
        return;
    }
    memset(results, 0, sizeof(*results) * RADAR_ZONE_COUNT);
    if (config == NULL || targets == NULL) {
        return;
    }

    for (size_t zone = 0U; zone < RADAR_ZONE_COUNT; ++zone) {
        const radar_detection_zone_config_t *zone_config =
            &config->zones[zone];
        if (zone_config->point_count < 3U ||
            zone_config->point_count > RADAR_ZONE_MAX_POINTS) {
            continue;
        }

        for (size_t target = 0U; target < target_count; ++target) {
            if (!targets[target].present ||
                !radar_polygon_contains_mm(zone_config->points,
                                           zone_config->point_count,
                                           targets[target].x_mm,
                                           targets[target].y_mm)) {
                continue;
            }

            if (results[zone].target_count < UINT8_MAX) {
                ++results[zone].target_count;
            }
            int32_t speed_cms = targets[target].speed_cms;
            if (speed_cms < 0) {
                speed_cms = -speed_cms;
            }
            if ((uint32_t)speed_cms >
                zone_config->movement_threshold_cms) {
                results[zone].movement = true;
            }
        }
        results[zone].presence = results[zone].target_count > 0U;
    }
}
