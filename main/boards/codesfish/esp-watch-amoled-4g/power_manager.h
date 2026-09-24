#pragma once
#include <vector>
#include <functional>
#include <esp_timer.h>
#include <driver/gpio.h>
#include "cw2015.h"
#include "esp_log.h"
#include <memory.h>

class PowerManager {
private:
    esp_timer_handle_t timer_handle_;
    std::function<void(bool)> on_charging_status_changed_;
    std::function<void(bool)> on_low_battery_status_changed_;

    uint32_t battery_level_ = 0;
    
    float last_voltage = 0.0f;
    float soc, voltage;
    bool is_charging_ = false;
    bool is_low_battery_ = false;
    int ticks_ = 0;
    const int kBatteryInterval = 60;
    const int kLowBatteryLevel = 20;

    CW2015* cw2015_ = nullptr;
    bool cw2015_initialized_ = false;

    void CheckBatteryStatus() {
        if (!cw2015_initialized_) {
            return;
        }
        
        bool new_charging_status = IsCharging();
        if (new_charging_status != is_charging_) {
            is_charging_ = new_charging_status;
            if (on_charging_status_changed_) {
                on_charging_status_changed_(is_charging_);
            }
            ReadBatteryData();
            return;
        }
        bool new_low_battery_status = battery_level_ <= kLowBatteryLevel;
        if (new_low_battery_status != is_low_battery_) {
            is_low_battery_ = new_low_battery_status;
            if (on_low_battery_status_changed_) {
                on_low_battery_status_changed_(is_low_battery_);
            }
        }
        if (ticks_ % kBatteryInterval == 0) {
            ReadBatteryData();
        }
        ticks_++;
    }

    void ReadBatteryData() {
        if (cw2015_ && cw2015_initialized_) {
            soc = cw2015_->capacity();
            ESP_LOGI("PowerManager", "----------电池电量: %.2f %%", (double)soc);
            if (soc < 0) soc = 0;
            if (soc > 100) soc = 100;
            battery_level_ = (uint32_t)soc;

            voltage = cw2015_->voltage();
            ESP_LOGI("PowerManager", "----------电池电压: %.3fV", (double)voltage);
            last_voltage = voltage;
        }
    }

public:
    PowerManager(i2c_master_bus_handle_t i2c_bus) {
        cw2015_ = new CW2015(i2c_bus, 0x62);
        if (cw2015_) {
            cw2015_->qstart();
            cw2015_initialized_ = true;
            ESP_LOGI("PowerManager", "CW2015 initialized successfully");
        } else {
            ESP_LOGW("PowerManager", "CW2015 allocation failed");
        }

        if (!cw2015_initialized_) {
            ESP_LOGW("PowerManager", "CW2015 not available, battery management will be disabled");
        }

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
        esp_err_t err = esp_timer_create(&timer_args, &timer_handle_);
        if (err != ESP_OK) {
            ESP_LOGE("PowerManager", "Timer creation failed: %s", esp_err_to_name(err));
        }
        err = esp_timer_start_periodic(timer_handle_, 1000000);
        if (err != ESP_OK) {
            ESP_LOGE("PowerManager", "Timer start failed: %s", esp_err_to_name(err));
        }
    }

    ~PowerManager() {
        if (timer_handle_) {
            esp_timer_stop(timer_handle_);
            esp_timer_delete(timer_handle_);
        }
        if (cw2015_) {
            delete cw2015_;
        }
    }

    bool IsCharging() {
        is_charging_ = false;
        if (cw2015_ && cw2015_initialized_) {
            is_charging_ = cw2015_->isCharging();
        }
        return is_charging_;
    }

    bool IsDischarging() {
        return !is_charging_;
    }

    uint8_t GetBatteryLevel() {
        if (!cw2015_initialized_) {
            return 0;
        }
        return battery_level_;
    }

    void OnLowBatteryStatusChanged(std::function<void(bool)> callback) {
        on_low_battery_status_changed_ = callback;
    }

    void OnChargingStatusChanged(std::function<void(bool)> callback) {
        on_charging_status_changed_ = callback;
    }
};