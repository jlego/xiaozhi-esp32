#include <stdio.h>
#include "max17048.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "MAX17048_COMP";

// Register Addresses
#define MAX17048_VCELL_REG 0x02
#define MAX17048_SOC_REG 0x04
#define MAX17048_VERSION_REG 0x08
#define MAX17048_CRATE_REG 0x16
#define MAX17048_CMD_REG 0xFE

// Global variables
static i2c_master_dev_handle_t i2c_dev_handle = NULL;
static bool is_initialized = false;
static max17048_config_t current_config;

// --- Internal Helper Functions ---
static esp_err_t max17048_write_word(uint8_t reg_addr, uint16_t data)
{
    if (!is_initialized || i2c_dev_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // uint8_t write_buf[3] = {reg_addr, (data >> 8) & 0xFF, data & 0xFF};
    uint8_t write_buf[3];
    write_buf[0] = reg_addr;
    write_buf[1] = static_cast<uint8_t>((data >> 8) & 0xFF);
    write_buf[2] = static_cast<uint8_t>(data & 0xFF);
    uint32_t timeout_ms = current_config.i2c_timeout_ms;
    return i2c_master_transmit(i2c_dev_handle, write_buf, sizeof(write_buf), timeout_ms);
}

static esp_err_t max17048_read_word(uint8_t reg_addr, uint16_t *data)
{
    if (!is_initialized || i2c_dev_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t read_buf[2];
    uint32_t timeout_ms = current_config.i2c_timeout_ms;
    esp_err_t ret = i2c_master_transmit_receive(i2c_dev_handle, &reg_addr, 1, read_buf, 2, timeout_ms);
    if (ret == ESP_OK)
    {
        *data = (read_buf[0] << 8) | read_buf[1];
    }
    return ret;
}

// --- Public API Functions ---

void max17048_get_default_config(max17048_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->i2c_bus_handle = NULL;  // Must be set by caller
    config->device_address = 0x36;  // MAX17048 I2C address
    config->i2c_freq_hz = 100000;   // 100kHz frequency
    config->i2c_timeout_ms = 1000;  // 1000ms timeout
}

esp_err_t max17048_init_with_config(const max17048_config_t *config)
{
    if (config == NULL || config->i2c_bus_handle == NULL)
    {
        ESP_LOGE(TAG, "Configuration pointer or bus handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (is_initialized)
    {
        ESP_LOGW(TAG, "MAX17048 already initialized.");
        return ESP_OK;
    }

    // Store configuration
    current_config = *config;

    // Create I2C device handle
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->device_address,
        .scl_speed_hz = config->i2c_freq_hz,
    };

    esp_err_t err = i2c_master_bus_add_device(config->i2c_bus_handle, &dev_cfg, &i2c_dev_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        return err;
    }

    is_initialized = true;

    // Check if device is present
    uint16_t version;
    err = max17048_get_version(&version);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "MAX17048 not found on I2C bus.");
        is_initialized = false;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "MAX17048 found. Version: 0x%04X", version);

    return ESP_OK;
}

esp_err_t max17048_init_on_bus_with_config(const max17048_config_t *config)
{
    // For new I2C master API, this is the same as init_with_config
    // since bus is already initialized externally
    return max17048_init_with_config(config);
}

esp_err_t max17048_init(void)
{
    ESP_LOGW(TAG, "Using legacy init function. Consider using max17048_init_with_config() instead.");
    ESP_LOGE(TAG, "Legacy init function not supported with new I2C master API. Use max17048_init_with_config() instead.");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t max17048_init_custom(i2c_port_t i2c_num, gpio_num_t sda_pin, gpio_num_t scl_pin)
{
    ESP_LOGW(TAG, "Using legacy init_custom function. Consider using max17048_init_with_config() instead.");
    ESP_LOGE(TAG, "Legacy init functions not supported with new I2C master API. Use max17048_init_with_config() instead.");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t max17048_init_on_bus(i2c_port_t i2c_num)
{
    ESP_LOGW(TAG, "Using legacy init_on_bus function. Consider using max17048_init_on_bus_with_config() instead.");
    ESP_LOGE(TAG, "Legacy init functions not supported with new I2C master API. Use max17048_init_on_bus_with_config() instead.");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t max17048_get_soc(float *soc)
{
    uint16_t raw_soc;
    esp_err_t ret = max17048_read_word(MAX17048_SOC_REG, &raw_soc);
    if (ret == ESP_OK)
    {
        *soc = (raw_soc >> 8) + ((raw_soc & 0xFF) / 256.0f);
    }
    return ret;
}

esp_err_t max17048_get_voltage(float *voltage)
{
    uint16_t raw_voltage;
    esp_err_t ret = max17048_read_word(MAX17048_VCELL_REG, &raw_voltage);
    if (ret == ESP_OK)
    {
        *voltage = raw_voltage * 0.000078125f; // LSB = 78.125uV
    }
    return ret;
}

esp_err_t max17048_get_crate(float *crate)
{
    uint16_t raw_crate;
    esp_err_t ret = max17048_read_word(MAX17048_CRATE_REG, &raw_crate);
    if (ret == ESP_OK)
    {
        int16_t signed_crate = (int16_t)raw_crate;
        *crate = signed_crate * 0.208f; // LSB = 0.208%/hr
    }
    return ret;
}

esp_err_t max17048_get_version(uint16_t *version)
{
    return max17048_read_word(MAX17048_VERSION_REG, version);
}

esp_err_t max17048_reset(void)
{
    return max17048_write_word(MAX17048_CMD_REG, 0x5400);
}