#include "sleep_timer.h"
#include "application.h"
#include "board.h"
#include "display.h"
#include "settings.h"

#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_lvgl_port.h>
#include <esp_idf_version.h>
#include <inttypes.h>

#define TAG "SleepTimer"


SleepTimer::SleepTimer(int seconds_to_light_sleep, int seconds_to_deep_sleep)
    : seconds_to_light_sleep_(seconds_to_light_sleep), seconds_to_deep_sleep_(seconds_to_deep_sleep) {
    esp_timer_create_args_t timer_args = {
        .callback = [](void* arg) {
            auto self = static_cast<SleepTimer*>(arg);
            self->CheckTimer();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "sleep_timer",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &sleep_timer_));
}

SleepTimer::~SleepTimer() {
    esp_timer_stop(sleep_timer_);
    esp_timer_delete(sleep_timer_);
}

void SleepTimer::SetEnabled(bool enabled) {
    if (enabled && !enabled_) {
        Settings settings("wifi", false);
        if (!settings.GetBool("sleep_mode", true)) {
            ESP_LOGI(TAG, "Power save timer is disabled by settings");
            return;
        }

        ticks_ = 0;
        enabled_ = enabled;
        ESP_ERROR_CHECK(esp_timer_start_periodic(sleep_timer_, 1000000));
        ESP_LOGI(TAG, "Sleep timer enabled");
    } else if (!enabled && enabled_) {
        ESP_ERROR_CHECK(esp_timer_stop(sleep_timer_));
        enabled_ = enabled;
        WakeUp();
        ESP_LOGI(TAG, "Sleep timer disabled");
    }
}

void SleepTimer::OnEnterLightSleepMode(std::function<void()> callback) {
    on_enter_light_sleep_mode_ = callback;
}

void SleepTimer::OnExitLightSleepMode(std::function<void()> callback) {
    on_exit_light_sleep_mode_ = callback;
}

void SleepTimer::OnEnterDeepSleepMode(std::function<void()> callback) {
    on_enter_deep_sleep_mode_ = callback;
}

void SleepTimer::CheckTimer() {
    auto& app = Application::GetInstance();
    if (!app.CanEnterSleepMode()) {
        ticks_ = 0;
        return;
    }

    ticks_++;
    if (seconds_to_light_sleep_ != -1 && ticks_ >= seconds_to_light_sleep_) {
        if (!in_light_sleep_mode_) {
            in_light_sleep_mode_ = true;
            if (on_enter_light_sleep_mode_) {
                on_enter_light_sleep_mode_();
            }

            auto& audio_service = app.GetAudioService();
            bool is_wake_word_running = audio_service.IsWakeWordRunning();
            if (is_wake_word_running) {
                audio_service.EnableWakeWordDetection(false);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        
            app.Schedule([this, &app]() {
                // 获取LVGL任务句柄
                TaskHandle_t lvgl_task_handle = xTaskGetHandle("taskLVGL");
                
                while (in_light_sleep_mode_) {
                    // 暂停LVGL任务
                    if (lvgl_task_handle != NULL) {
                        vTaskSuspend(lvgl_task_handle);
                    }
                    lv_refr_now(nullptr);
                    lvgl_port_stop();

                    // 暂停所有LVGL定时器
                    lv_timer_enable(false);
                    // 获取板级特定的闹钟管理器
                    auto& board = Board::GetInstance();
                    int64_t alarm_wakeup_time_us = -1;
                    
                    // 尝试获取板级闹钟管理器
                    alarm_wakeup_time_us = board.GetNextAlarmWakeupTimeUs();
                    
                    // 只有在闹钟时间有效时才设置唤醒时间
                    if (alarm_wakeup_time_us > 0) {
                        ESP_LOGI(TAG, "Setting alarm wakeup time: %lld us", (long long)alarm_wakeup_time_us);
                        // 配置timer唤醒源
                        esp_sleep_enable_timer_wakeup(alarm_wakeup_time_us);
                    } else {
                        ESP_LOGI(TAG, "No valid alarm found, not setting timer wakeup");
                    }
                    esp_sleep_enable_gpio_wakeup();
                    
                    // 根据芯片类型选择不同的启动按钮GPIO
#if CONFIG_IDF_TARGET_ESP32C3
                    gpio_wakeup_enable(GPIO_NUM_9, GPIO_INTR_LOW_LEVEL);  // ESP32-C3 默认启动按钮
#elif CONFIG_IDF_TARGET_ESP32S3
                    gpio_wakeup_enable(GPIO_NUM_0, GPIO_INTR_LOW_LEVEL);  // ESP32-S3 默认启动按钮
#else
                    gpio_wakeup_enable(GPIO_NUM_0, GPIO_INTR_LOW_LEVEL);  // 默认使用GPIO0
#endif          
                    // 进入light sleep模式
                    esp_light_sleep_start();
                    
                    uint32_t wakeup_causes = esp_sleep_get_wakeup_causes();
                    ESP_LOGI(TAG, "Wake up from light sleep, wakeup_causes: 0x%x", wakeup_causes);
                    if (wakeup_causes & (1U << ESP_SLEEP_WAKEUP_GPIO)) {
                        ESP_LOGI(TAG, "GPIO wake up detected, exiting sleep loop");
                        break;
                    }else if (wakeup_causes & (1U << ESP_SLEEP_WAKEUP_TIMER)) {
                        ESP_LOGI(TAG, "Timer wake up detected, checking if alarm check is needed");
                        auto& board = Board::GetInstance();
                        if (board.GetNearestAlarm() != nullptr) {
                            ESP_LOGI(TAG, "有闹钟需要检查，触发闹钟检查");
                            board.TriggerAlarmCheck();
                        } else {
                            ESP_LOGI(TAG, "没有闹钟需要检查，跳过闹钟检查");
                        }
                        break;
                    }
                }
                // 恢复LVGL定时器
                lv_timer_enable(true);
                lvgl_port_resume();
                // 恢复LVGL任务
                if (lvgl_task_handle != NULL) {
                    vTaskResume(lvgl_task_handle);
                }
                // 添加延迟确保LVGL任务完全恢复
                vTaskDelay(pdMS_TO_TICKS(10));
                
                WakeUp();

            });

            if (is_wake_word_running) {
                audio_service.EnableWakeWordDetection(true);
            }
        }
    }
    if (seconds_to_deep_sleep_ != -1 && ticks_ >= seconds_to_deep_sleep_) {
        if (on_enter_deep_sleep_mode_) {
            on_enter_deep_sleep_mode_();
        }

        esp_deep_sleep_start();
    }
}

void SleepTimer::WakeUp() {
    ticks_ = 0;
    if (in_light_sleep_mode_) {
        in_light_sleep_mode_ = false;
        if (on_exit_light_sleep_mode_) {
            on_exit_light_sleep_mode_();
        }
    }
}