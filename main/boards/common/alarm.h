#ifndef ALARM_H
#define ALARM_H

#include <string>
#include <vector>
#include <functional>
#include <ctime>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "nvs.h"
#include "nvs_flash.h"

// 闹钟重复模式
enum class AlarmRepeat {
    ONCE,        // 一次性
    DAILY,       // 每天
    WEEKLY,      // 每周一次
    WEEKDAYS     // 工作日（周一到周五）
};

// 闹钟结构体
struct Alarm {
    int id;
    int hour;
    int minute;
    AlarmRepeat repeat;
    std::string note;
    bool enabled;
    
    Alarm() : id(0), hour(0), minute(0), repeat(AlarmRepeat::ONCE), enabled(true) {}
    Alarm(int id, int hour, int minute, AlarmRepeat repeat, const std::string& note) 
        : id(id), hour(hour), minute(minute), repeat(repeat), note(note), enabled(true) {}
};

class AlarmManager {
private:
    static const char* TAG;
    std::vector<Alarm> alarms_;
    TimerHandle_t check_timer_;
    std::function<void(const Alarm&)> alarm_callback_;
    
    // 添加防重复触发机制
    struct LastTriggeredAlarm {
        int id;
        time_t trigger_time;  // 上次触发的时间（分钟级别）
    };
    LastTriggeredAlarm last_triggered_alarm_;
    
    // 从NVS加载闹钟数据
    esp_err_t LoadAlarmsFromNVS();
    
    // 保存闹钟数据到NVS
    esp_err_t SaveAlarmsToNVS();
    
    // 检查是否有闹钟需要触发
    void CheckAlarms();
    
    // 定时器回调函数
    static void TimerCallback(TimerHandle_t xTimer);
    
    // 获取下一个闹钟时间
    time_t GetNextAlarmTime(const Alarm& alarm) const;
    
    // 检查闹钟是否应该今天触发
    bool ShouldTriggerToday(const Alarm& alarm, const tm* current_time) const;

public:
    AlarmManager();
    ~AlarmManager();
    
    // 初始化闹钟管理器
    esp_err_t Init();
    
    // 添加闹钟
    esp_err_t AddAlarm(const Alarm& alarm);
    
    // 删除闹钟
    esp_err_t RemoveAlarm(int id);
    
    // 更新闹钟
    esp_err_t UpdateAlarm(const Alarm& alarm);
    
    // 获取所有闹钟
    const std::vector<Alarm>& GetAlarms() const { return alarms_; }
    
    // 设置闹钟触发回调
    void SetAlarmCallback(std::function<void(const Alarm&)> callback);
    
    // 启用/禁用闹钟
    esp_err_t EnableAlarm(int id, bool enable);
    
    // 获取最近的闹钟
    Alarm* GetNearestAlarm();
    
    // 获取下一个闹钟的唤醒时间（用于浅睡眠唤醒）
    int64_t GetNextAlarmWakeupTimeUs() const;
    
    // 公共函数用于触发闹钟检查（供外部调用）
    void TriggerAlarmCheck();
};

#endif // ALARM_H