#pragma once
#include <vector>
#include <functional>
#include <esp_timer.h>
#include <driver/gpio.h>
// #include <esp_adc/adc_oneshot.h>
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
    const int kBatteryDataCount = 3;
    const int kLowBatteryLevel = 20;

    CW2015* cw2015_ = nullptr;

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
        if (cw2015_) {
            soc = cw2015_->capacity(); // 直接获取百分比值，不需要再乘以100
            ESP_LOGI("PowerManager", "----------电池电量: %.2f %%", (double)soc);
            if (soc < 0) soc = 0;
            if (soc > 100) soc = 100;
            battery_level_ = (soc / 20) * 20;

            voltage = cw2015_->voltage();
            ESP_LOGI("PowerManager", "----------电池电压: %.3fV", (double)voltage);
            last_voltage = voltage;
        }
    }

public:
    PowerManager(i2c_master_bus_handle_t i2c_bus) {
        // Initialize CW2015 fuel gauge
        cw2015_ = new CW2015(i2c_bus, 0x62);  // CW2015 I2C address
        if (cw2015_) {
            cw2015_->qstart();
        } else {
            ESP_LOGE("PowerManager", "CW2015 initialization failed");
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
        esp_err_t err = esp_timer_create(&timer_args, &timer_handle_);
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
        if (cw2015_) {
            delete cw2015_;
        }
    }

    bool IsCharging() {
        is_charging_ = false;
        if (cw2015_) {
            is_charging_ = cw2015_->isCharging();
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
