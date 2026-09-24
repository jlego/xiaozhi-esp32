#ifndef MAX17048_H
#define MAX17048_H

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MAX17048 runtime configuration structure
 */
typedef struct {
    i2c_master_bus_handle_t i2c_bus_handle;  // I2C master bus handle
    uint16_t device_address;                  // Device I2C address (default: 0x36)
    uint32_t i2c_freq_hz;                     // I2C frequency (default: 100000)
    uint32_t i2c_timeout_ms;                  // I2C timeout (default: 1000)
} max17048_config_t;

/**
 * @brief Get default configuration for MAX17048.
 *
 * This function fills the configuration structure with default values
 * that match the previous Kconfig defaults.
 *
 * @param config Pointer to configuration structure to fill with defaults.
 */
void max17048_get_default_config(max17048_config_t *config);

/**
 * @brief Initialize the MAX17048 fuel gauge component with runtime configuration.
 *
 * @param config Pointer to configuration structure.
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL if initialization fails or device not found
 */
esp_err_t max17048_init_with_config(const max17048_config_t *config);

/**
 * @brief Initialize the MAX17048 fuel gauge on an already configured I2C bus.
 *
 * This function is used when the I2C bus is managed externally.
 * It verifies communication with the device without installing or
 * configuring the I2C driver.
 *
 * @param config Pointer to configuration structure.
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL if device not found
 */
esp_err_t max17048_init_on_bus_with_config(const max17048_config_t *config);

/**
 * @brief Initialize the MAX17048 fuel gauge component using settings from menuconfig.
 *
 * @deprecated Use max17048_init_with_config() instead for runtime configuration
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL if initialization fails or device not found
 */
esp_err_t max17048_init(void);

/**
 * @brief Initialize the MAX17048 fuel gauge component with custom parameters.
 *
 * This function allows overriding the default settings from menuconfig.
 *
 * @param i2c_num The I2C port number to use.
 * @param sda_pin The GPIO number for the SDA line.
 * @param scl_pin The GPIO number for the SCL line.
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL if initialization fails or device not found
 */
esp_err_t max17048_init_custom(i2c_port_t i2c_num, gpio_num_t sda_pin, gpio_num_t scl_pin);

/**
 * @brief Initialize the MAX17048 fuel gauge on an already configured I2C bus.
 *
 * This function is used when the I2C bus is managed externally.
 * It verifies communication with the device without installing or
 * configuring the I2C driver.
 *
 * @param i2c_num I2C port number that is already initialized.
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL if device not found
 */
esp_err_t max17048_init_on_bus(i2c_port_t i2c_num);

/**
 * @brief Get the battery's state of charge (SOC) as a percentage.
 *
 * @param soc Pointer to a float where the SOC will be stored.
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL if read fails
 */
esp_err_t max17048_get_soc(float *soc);

/**
 * @brief Get the battery's cell voltage.
 *
 * @param voltage Pointer to a float where the voltage will be stored.
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL if read fails
 */
esp_err_t max17048_get_voltage(float *voltage);

/**
 * @brief Get the battery's charge or discharge rate.
 *
 * @param crate Pointer to a float where the rate (%/hour) will be stored.
 *                A positive value indicates charging, negative indicates discharging.
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL if read fails
 */
esp_err_t max17048_get_crate(float *crate);

/**
 * @brief Get the production version of the IC.
 *
 * @param version Pointer to a uint16_t to store the version number.
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL if read fails
 */
esp_err_t max17048_get_version(uint16_t *version);

/**
 * @brief Send a Power-On Reset (POR) command to the device.
 *
 * This will reset all registers to their default values.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL if write fails
 */
esp_err_t max17048_reset(void);

#ifdef __cplusplus
}
#endif

#endif // MAX17048_H