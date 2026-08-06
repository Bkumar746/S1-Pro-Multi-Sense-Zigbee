#include "i2c_bus.h"

#include "esp_check.h"
#include "s1_pro_config.h"

static const char *TAG = "i2c_bus";
static i2c_master_bus_handle_t s_bus;

esp_err_t s1_pro_i2c_bus_init(void)
{
    if (s_bus != NULL) {
        return ESP_OK;
    }
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = S1_PRO_I2C_PORT,
        .sda_io_num = S1_PRO_I2C_SDA_GPIO,
        .scl_io_num = S1_PRO_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = false,
        },
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_bus), TAG,
                        "Failed to create shared I2C bus");
    return ESP_OK;
}

i2c_master_bus_handle_t s1_pro_i2c_bus_get(void)
{
    return s_bus;
}

esp_err_t s1_pro_i2c_add_device(uint16_t address,
                                i2c_master_dev_handle_t *device)
{
    ESP_RETURN_ON_FALSE(s_bus != NULL && device != NULL, ESP_ERR_INVALID_STATE,
                        TAG, "I2C bus is not initialized");
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = S1_PRO_I2C_FREQUENCY_HZ,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };
    return i2c_master_bus_add_device(s_bus, &device_config, device);
}
