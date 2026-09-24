#include "cw2015.h"
#include <cmath>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "CW2015"

#define REG_VERSION 0x00
#define REG_VCELL   0x02
#define REG_SOC     0x04
#define REG_ALERT   0x06
#define REG_CONFIG  0x08
#define REG_MODE    0x0A

CW2015::CW2015(i2c_master_bus_handle_t i2c_bus, uint8_t i2c_addr) 
    : I2cDevice(i2c_bus, i2c_addr) 
{
    uint8_t version = ReadReg(REG_VERSION);
    ESP_LOGI(TAG, "CW2015 Chip Version: 0x%02X", version);
    
    qstart();
    
    vTaskDelay(pdMS_TO_TICKS(100));
}

float CW2015::voltage() {
    uint8_t buffer[2];
    ReadRegs(REG_VCELL, buffer, 2);
    uint16_t raw = (buffer[0] << 8) | buffer[1];
    return raw * 0.305f / 1000.0f;
}

float CW2015::capacity() {
    uint8_t buffer[2];
    ReadRegs(REG_SOC, buffer, 2);
    
    uint8_t soc_integer = buffer[0];
    uint8_t soc_decimal = buffer[1];
    
    float calculated_soc = soc_integer + (float)soc_decimal / 256.0f;
    
    if (calculated_soc > 100.0f) {
        return 100.0f;
    } else if (calculated_soc < 0.0f) {
        return 0.0f;
    }
    
    return calculated_soc;
}

bool CW2015::isCharging() {
    static float last_voltage = 0.0f;
    float current_voltage = voltage();
    bool is_charging = false;
    
    if (last_voltage > 0.0f) {
        if (current_voltage > last_voltage + 0.01f && current_voltage > 3.8f) {
            is_charging = true;
        }
    }
    
    last_voltage = current_voltage;
    return is_charging;
}

void CW2015::qstart() {
    ESP_LOGI(TAG, "Executing quick start...");
    WriteReg(REG_MODE, 0x00);
}

void CW2015::goSleep() {
    WriteReg(REG_MODE, 0x11);
}