#pragma once

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t s1_pro_i2c_bus_init(void);
i2c_master_bus_handle_t s1_pro_i2c_bus_get(void);
esp_err_t s1_pro_i2c_add_device(uint16_t address,
                                i2c_master_dev_handle_t *device);

#ifdef __cplusplus
}
#endif
