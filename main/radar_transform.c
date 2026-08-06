#include "radar_transform.h"

#include <limits.h>

int16_t radar_transform_x_mm(int16_t x_mm, bool flip_y_axis)
{
    if (!flip_y_axis) {
        return x_mm;
    }
    return x_mm == INT16_MIN ? INT16_MAX : (int16_t)-x_mm;
}
