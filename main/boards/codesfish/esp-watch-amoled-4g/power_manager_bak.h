#pragma once
#include <vector>
#include <functional>
#include <esp_timer.h>
#include <driver/gpio.h>
// #include <esp_adc/adc_oneshot.h>
#include "max17048.h"
#include "esp_log.h"
#include <memory.h>

class PowerManager {
private:
    esp_timer_handle_t timer_handle_;
    std::function<void(bool)> on_charging_status_changed_;
    std::function<void(bool)> on_low_battery_status_changed_;

    uint32_t battery_level_ = 0;
    
    float last_voltage = 0.0f;
    float soc, voltage, charge_rate;
    bool is_charging_ = false;
    bool is_low_battery_ = false;
    int ticks_ = 0;
    const int kBatteryInterval = 60;
    const int kBatteryDataCount = 3;
    const int kLowBatteryLevel = 20;

    void CheckBatteryStatus() {
        // Get charging status
        bool new_charging_status = IsCharging();
        if (new_charging_status != is_charging_) {
            is_charging_ = new_charging_status;
            if (on_charging_status_changed_) {
                on_charging_status_changed_(is_charging_);
            }
            ReadBatteryData();
            return;
        }
        // Check low battery status
        bool new_low_battery_status = battery_level_ <= kLowBatteryLevel;
        if (new_low_battery_status != is_low_battery_) {
            is_low_battery_ = new_low_battery_status;
            if (on_low_battery_status_changed_) {
                on_low_battery_status_changed_(is_low_battery_);
            }
        }
        // 如果电池电量数据充足，则每 kBatteryAdcInterval 个 tick 读取一次电池电量数据
        if (ticks_ % kBatteryInterval == 0) {
            ReadBatteryData();
        }
        ticks_++;
    }

    void ReadBatteryData() {
        if (max17048_get_soc(&soc) == ESP_OK) {
            ESP_LOGI("PowerManager", "----------电池电量: %.2f %%", (double)soc);
            if (soc < 0) soc = 0;
            if (soc > 100) soc = 100;
            battery_level_ = (soc / 20) * 20;
        }
        if (max17048_get_voltage(&voltage) == ESP_OK) {
            ESP_LOGI("PowerManager", "----------电池电压: %.3fV", (double)voltage);
            // if(voltage - last_voltage > 0.01f){
            //     ESP_LOGI("PowerManager", "充电中");
            // }
            last_voltage = voltage;
        }
    }

public:
    PowerManager(i2c_master_bus_handle_t i2c_bus) {
        max17048_config_t max_config;
        max17048_get_default_config(&max_config);
        max_config.i2c_bus_handle = i2c_bus;
        max_config.device_address = 0x36;  // MAX17048 I2C address
        max_config.i2c_freq_hz = 100000;   // 100kHz I2C frequency
        
        // Initialize fuel gauge
        esp_err_t err = max17048_init_on_bus_with_config(&max_config);
        if (err != ESP_OK) {
            ESP_LOGE("PowerManager", "MAX17048 initialization failed: %s", esp_err_to_name(err));
            // 不要崩溃，继续运行，只是电池管理功能不可用
        }

        // 创建1秒的电池电量检查定时器
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                PowerManager* self = static_cast<PowerManager*>(arg);
                self->CheckBatteryStatus();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "battery_check_timer",
            .skip_unhandled_events = true,
        };
        err = esp_timer_create(&timer_args, &timer_handle_);
        if (err != ESP_OK) {
            ESP_LOGE("PowerManager", "Timer creation failed: %s", esp_err_to_name(err));
        }
        err = esp_timer_start_periodic(timer_handle_, 1000000);
        if (err != ESP_OK) {
            ESP_LOGE("PowerManager", "Timer start failed: %s", esp_err_to_name(err));
        }
        // ReadBatteryData();
    }

    ~PowerManager() {
        if (timer_handle_) {
            esp_timer_stop(timer_handle_);
            esp_timer_delete(timer_handle_);
        }
    }

    bool IsCharging() {
        is_charging_ = false;
        if (max17048_get_crate(&charge_rate) == ESP_OK) {
            if (charge_rate > 1.0f) {
                // ESP_LOGI("PowerManager", "变化率: %.2f %%/hr", charge_rate);
                is_charging_ = true;
            }
        }
        return is_charging_;
    }

    bool IsDischarging() {
        // 没有区分充电和放电，所以直接返回相反状态
        return !is_charging_;
    }

    uint8_t GetBatteryLevel() {
        return battery_level_;
    }

    void OnLowBatteryStatusChanged(std::function<void(bool)> callback) {
        on_low_battery_status_changed_ = callback;
    }

    void OnChargingStatusChanged(std::function<void(bool)> callback) {
        on_charging_status_changed_ = callback;
    }
};
