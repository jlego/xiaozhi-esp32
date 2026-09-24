#include "alarm.h"
#include "cJSON.h"
#include <cstring>
#include "esp_system.h"
#include "esp_log.h"
#include <memory>      // 包含智能指针头文件

// 正确定义 AlarmManager 类的静态成员变量
const char* AlarmManager::TAG = "AlarmManager";

AlarmManager::AlarmManager() 
    : check_timer_(nullptr) {
    // 初始化已在初始化列表中完成
    ESP_LOGI(TAG, "AlarmManager构造函数被调用");
    
    // 初始化防重复触发机制
    last_triggered_alarm_.id = -1;
    last_triggered_alarm_.trigger_time = 0;
}

AlarmManager::~AlarmManager() {
    if (check_timer_) {
        xTimerDelete(check_timer_, portMAX_DELAY);
    }
}

esp_err_t AlarmManager::Init() {
    // 初始化NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS分区已满或版本不匹配，擦除并重新初始化
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    ESP_LOGI(TAG, "NVS初始化完成");
    
    // 从NVS加载闹钟数据
    err = LoadAlarmsFromNVS();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load alarms from NVS: %s", esp_err_to_name(err));
    }
    
    // 创建定时器，每分钟检查一次闹钟
    check_timer_ = xTimerCreate(
        "AlarmCheckTimer",
        pdMS_TO_TICKS(60000), // 1分钟
        pdTRUE, // 自动重载
        this,   // 定时器ID
        TimerCallback
    );
    
    if (check_timer_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create alarm check timer");
        return ESP_FAIL;
    }
    
    // 启动定时器
    if (xTimerStart(check_timer_, portMAX_DELAY) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start alarm check timer");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "AlarmManager初始化成功，定时器已启动");
    return ESP_OK;
}

esp_err_t AlarmManager::LoadAlarmsFromNVS() {
    // 修改为兼容 wifi_configuration_ap.cc 中的数据格式
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("alarm", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %d", err);
        return err;
    }
    
    // 读取alarms数据
    size_t alarms_size = 0;
    err = nvs_get_blob(nvs, "alarms", NULL, &alarms_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to get alarms size: %d", err);
        nvs_close(nvs);
        return err;
    } else if (err == ESP_ERR_NVS_NOT_FOUND || alarms_size == 0) {
        // 如果没有alarms数据，初始化为空数组
        nvs_close(nvs);
        alarms_.clear();
        ESP_LOGI(TAG, "NVS中没有闹钟数据");
        return ESP_OK;
    } else {
        // 分配内存并读取alarms数据
        char *alarms_data = (char *)malloc(alarms_size);
        if (!alarms_data) {
            ESP_LOGE(TAG, "Failed to allocate memory for alarms data");
            nvs_close(nvs);
            return ESP_ERR_NO_MEM;
        }
        
        err = nvs_get_blob(nvs, "alarms", alarms_data, &alarms_size);
        nvs_close(nvs);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read alarms data: %d", err);
            free(alarms_data);
            return err;
        }
        
        // 解析alarms数据
        cJSON *alarms_json = cJSON_Parse(alarms_data);
        free(alarms_data);
        if (!alarms_json) {
            ESP_LOGE(TAG, "Failed to parse alarms JSON");
            return ESP_FAIL;
        }
        
        // 清空现有闹钟
        alarms_.clear();
        
        // 解析闹钟数组
        if (cJSON_IsArray(alarms_json)) {
            int array_size = cJSON_GetArraySize(alarms_json);
            ESP_LOGI(TAG, "解析到 %d 个闹钟", array_size);
            for (int i = 0; i < array_size; i++) {
                cJSON* alarm_item = cJSON_GetArrayItem(alarms_json, i);
                if (alarm_item != nullptr) {
                    Alarm alarm;
                    
                    cJSON* id_item = cJSON_GetObjectItemCaseSensitive(alarm_item, "id");
                    if (id_item != nullptr && cJSON_IsNumber(id_item)) {
                        alarm.id = id_item->valueint;
                    }
                    
                    cJSON* hour_item = cJSON_GetObjectItemCaseSensitive(alarm_item, "hour");
                    if (hour_item != nullptr && cJSON_IsNumber(hour_item)) {
                        alarm.hour = hour_item->valueint;
                    }
                    
                    cJSON* minute_item = cJSON_GetObjectItemCaseSensitive(alarm_item, "minute");
                    if (minute_item != nullptr && cJSON_IsNumber(minute_item)) {
                        alarm.minute = minute_item->valueint;
                    }
                    
                    cJSON* repeat_item = cJSON_GetObjectItemCaseSensitive(alarm_item, "repeat");
                    if (repeat_item != nullptr && cJSON_IsString(repeat_item)) {
                        std::string repeat_str = repeat_item->valuestring;
                        if (repeat_str == "once") {
                            alarm.repeat = AlarmRepeat::ONCE;
                        } else if (repeat_str == "daily") {
                            alarm.repeat = AlarmRepeat::DAILY;
                        } else if (repeat_str == "weekly") {
                            alarm.repeat = AlarmRepeat::WEEKLY;
                        } else if (repeat_str == "weekdays") {
                            alarm.repeat = AlarmRepeat::WEEKDAYS;
                        }
                    }
                    
                    cJSON* note_item = cJSON_GetObjectItemCaseSensitive(alarm_item, "note");
                    if (note_item != nullptr && cJSON_IsString(note_item)) {
                        alarm.note = note_item->valuestring;
                    }
                    
                    cJSON* enabled_item = cJSON_GetObjectItemCaseSensitive(alarm_item, "enabled");
                    if (enabled_item != nullptr) {
                        if (cJSON_IsBool(enabled_item)) {
                            alarm.enabled = cJSON_IsTrue(enabled_item);
                        }
                    } else {
                        // 默认启用
                        alarm.enabled = true;
                    }
                    
                    alarms_.push_back(alarm);
                    
                    // 打印每个解析的闹钟
                    ESP_LOGI(TAG, "加载闹钟: ID=%d, 时间=%02d:%02d, 重复=%d, 备注=%s, 启用=%s", 
                             alarm.id, alarm.hour, alarm.minute, 
                             static_cast<int>(alarm.repeat), alarm.note.c_str(),
                             alarm.enabled ? "是" : "否");
                }
            }
        }
        
        cJSON_Delete(alarms_json);
    }
    
    ESP_LOGI(TAG, "Loaded %d alarms from NVS", (int)alarms_.size());
    return ESP_OK;
}

esp_err_t AlarmManager::SaveAlarmsToNVS() {
    // 修改为兼容 wifi_configuration_ap.cc 中的数据格式
    
    // 创建闹钟数组
    cJSON* alarms_array = cJSON_CreateArray();
    
    // 添加闹钟数据到数组
    for (const auto& alarm : alarms_) {
        cJSON* alarm_item = cJSON_CreateObject();
        
        cJSON_AddNumberToObject(alarm_item, "id", alarm.id);
        cJSON_AddNumberToObject(alarm_item, "hour", alarm.hour);
        cJSON_AddNumberToObject(alarm_item, "minute", alarm.minute);
        
        const char* repeat_str = "";
        switch (alarm.repeat) {
            case AlarmRepeat::ONCE: repeat_str = "once"; break;
            case AlarmRepeat::DAILY: repeat_str = "daily"; break;
            case AlarmRepeat::WEEKLY: repeat_str = "weekly"; break;
            case AlarmRepeat::WEEKDAYS: repeat_str = "weekdays"; break;
        }
        cJSON_AddStringToObject(alarm_item, "repeat", repeat_str);
        
        cJSON_AddStringToObject(alarm_item, "note", alarm.note.c_str());
        cJSON_AddBoolToObject(alarm_item, "enabled", alarm.enabled);
        
        cJSON_AddItemToArray(alarms_array, alarm_item);
        
        // 打印每个保存的闹钟
        ESP_LOGI(TAG, "保存闹钟: ID=%d, 时间=%02d:%02d, 重复=%s, 备注=%s, 启用=%s", 
                 alarm.id, alarm.hour, alarm.minute, repeat_str, alarm.note.c_str(),
                 alarm.enabled ? "是" : "否");
    }
    
    // 转换为JSON字符串
    char* json_str = cJSON_PrintUnformatted(alarms_array);
    if (json_str == nullptr) {
        ESP_LOGE(TAG, "Failed to print alarms JSON");
        cJSON_Delete(alarms_array);
        return ESP_FAIL;
    }
    
    // 打印要保存到NVS的数据
    ESP_LOGI(TAG, "保存到NVS的闹钟数据: %s", json_str);
    
    // 使用与 wifi_configuration_ap.cc 兼容的方式保存到NVS
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("alarm", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing: %d", err);
        free(json_str);
        cJSON_Delete(alarms_array);
        return err;
    }
    
    err = nvs_set_blob(nvs, "alarms", json_str, strlen(json_str) + 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save alarms to NVS: %d", err);
        nvs_close(nvs);
        free(json_str);
        cJSON_Delete(alarms_array);
        return err;
    }
    
    err = nvs_commit(nvs);
    nvs_close(nvs);
    
    free(json_str);
    cJSON_Delete(alarms_array);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS changes: %d", err);
        return err;
    }
    
    ESP_LOGI(TAG, "Saved %d alarms to NVS", (int)alarms_.size());
    return ESP_OK;
}

void AlarmManager::CheckAlarms() {
    // 如果没有闹钟，直接返回
    if (alarms_.empty()) {
        ESP_LOGI(TAG, "没有闹钟需要检查");
        return;
    }
    
    time_t now = time(nullptr);
    tm current_time = {0};
    localtime_r(&now, &current_time);
    
    // 计算当前时间的分钟级别时间戳（用于防重复触发）
    time_t current_minute_time = now / 60 * 60;  // 四舍五入到分钟
    
    // 打印当前时间
    ESP_LOGI(TAG, "检查闹钟 - 当前时间: %04d-%02d-%02d %02d:%02d:%02d", 
             current_time.tm_year + 1900, current_time.tm_mon + 1, current_time.tm_mday,
             current_time.tm_hour, current_time.tm_min, current_time.tm_sec);
    
    for (auto& alarm : alarms_) {
        if (!alarm.enabled) {
            continue;
        }
        
        // 检查闹钟数据是否有效
        if (alarm.note.empty()) {
            ESP_LOGW(TAG, "闹钟备注为空，跳过检查: ID=%d", alarm.id);
            continue;
        }
        
        // 打印正在检查的闹钟
        ESP_LOGI(TAG, "检查闹钟: ID=%d, 时间=%02d:%02d, 重复=%d, 备注=%s", 
                 alarm.id, alarm.hour, alarm.minute, 
                 static_cast<int>(alarm.repeat), alarm.note.c_str());
        
        // 检查小时和分钟是否匹配
        if (current_time.tm_hour == alarm.hour && current_time.tm_min == alarm.minute) {
            // 检查是否应该今天触发
            if (ShouldTriggerToday(alarm, &current_time)) {
                // 检查是否在短时间内已经触发过这个闹钟（防重复触发）
                if (last_triggered_alarm_.id == alarm.id && 
                    last_triggered_alarm_.trigger_time == current_minute_time) {
                    ESP_LOGI(TAG, "闹钟已在此分钟内触发过，跳过: ID=%d", alarm.id);
                    continue;
                }
                
                ESP_LOGI(TAG, "Alarm triggered: %s (ID: %d)", alarm.note.c_str(), alarm.id);
                
                // 记录这次触发
                last_triggered_alarm_.id = alarm.id;
                last_triggered_alarm_.trigger_time = current_minute_time;
                
                // 调用回调函数
                if (alarm_callback_) {
                    alarm_callback_(alarm);
                }
                
                // 添加小延迟以避免看门狗超时
                vTaskDelay(pdMS_TO_TICKS(10));
                
                // 如果是一次性闹钟，禁用它
                if (alarm.repeat == AlarmRepeat::ONCE) {
                    alarm.enabled = false;
                    SaveAlarmsToNVS();
                }
            } else {
                ESP_LOGI(TAG, "闹钟今天不触发: ID=%d, 星期=%d", alarm.id, current_time.tm_wday);
            }
        }
    }
}

time_t AlarmManager::GetNextAlarmTime(const Alarm& alarm) const {
    time_t now = time(nullptr);
    tm current_time = {0};
    localtime_r(&now, &current_time);
    
    // 创建今天的闹钟时间
    tm alarm_time = current_time;
    alarm_time.tm_hour = alarm.hour;
    alarm_time.tm_min = alarm.minute;
    alarm_time.tm_sec = 0;
    
    // 检查今天是否应该触发闹钟
    if (ShouldTriggerToday(alarm, &current_time)) {
        time_t today_alarm = mktime(&alarm_time);
        // 如果今天的时间还没过，今天触发
        if (today_alarm > now) {
            return today_alarm;
        }
    }
    
    // 寻找下一个应该触发的日期
    tm next_date = current_time;
    for (int i = 1; i <= 8; i++) { // 最多检查8天
        next_date.tm_mday += 1;
        next_date.tm_isdst = -1; // 让 mktime 自动计算夏令时
        time_t next_timestamp = mktime(&next_date);
        
        // 检查这一天是否应该触发闹钟
        tm next_tm = {0};
        localtime_r(&next_timestamp, &next_tm);
        if (ShouldTriggerToday(alarm, &next_tm)) {
            next_tm.tm_hour = alarm.hour;
            next_tm.tm_min = alarm.minute;
            next_tm.tm_sec = 0;
            return mktime(&next_tm);
        }
    }
    
    // 如果8天内都没有找到合适的日期，返回一个很大的值
    return now + 8 * 24 * 60 * 60; // 8天后
}

bool AlarmManager::ShouldTriggerToday(const Alarm& alarm, const tm* current_time) const {
    switch (alarm.repeat) {
        case AlarmRepeat::ONCE:
        case AlarmRepeat::DAILY:
            return true;
            
        case AlarmRepeat::WEEKLY:
            // 每周一次，例如只在周日触发
            return (current_time->tm_wday == 0); // 0表示周日
            
        case AlarmRepeat::WEEKDAYS:
            // 工作日闹钟（周一到周五）
            return (current_time->tm_wday >= 1 && current_time->tm_wday <= 5);
            
        default:
            return false;
    }
}

void AlarmManager::TimerCallback(TimerHandle_t xTimer) {
    AlarmManager* manager = static_cast<AlarmManager*>(pvTimerGetTimerID(xTimer));
    if (manager) {
        // 只有在有闹钟的情况下才执行检查
        if (!manager->alarms_.empty()) {
            manager->CheckAlarms();
        } else {
            ESP_LOGI(manager->TAG, "没有闹钟，跳过定时器触发的闹钟检查");
        }
    }
}

esp_err_t AlarmManager::AddAlarm(const Alarm& alarm) {
    // 检查ID是否已存在
    for (const auto& existing_alarm : alarms_) {
        if (existing_alarm.id == alarm.id) {
            ESP_LOGE(TAG, "Alarm with ID %d already exists", alarm.id);
            return ESP_ERR_INVALID_ARG;
        }
    }
    
    alarms_.push_back(alarm);
    
    // 保存到NVS
    esp_err_t err = SaveAlarmsToNVS();
    if (err != ESP_OK) {
        // 如果保存失败，回滚
        alarms_.pop_back();
        return err;
    }
    
    ESP_LOGI(TAG, "Added alarm: %s (ID: %d)", alarm.note.c_str(), alarm.id);
    return ESP_OK;
}

esp_err_t AlarmManager::RemoveAlarm(int id) {
    for (auto it = alarms_.begin(); it != alarms_.end(); ++it) {
        if (it->id == id) {
            alarms_.erase(it);
            
            // 保存到NVS
            esp_err_t err = SaveAlarmsToNVS();
            if (err != ESP_OK) {
                // 注意：这里无法回滚删除操作
                return err;
            }
            
            ESP_LOGI(TAG, "Removed alarm with ID: %d", id);
            return ESP_OK;
        }
    }
    
    ESP_LOGW(TAG, "Alarm with ID %d not found", id);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t AlarmManager::UpdateAlarm(const Alarm& updated_alarm) {
    for (auto& alarm : alarms_) {
        if (alarm.id == updated_alarm.id) {
            alarm = updated_alarm;
            
            // 保存到NVS
            esp_err_t err = SaveAlarmsToNVS();
            if (err != ESP_OK) {
                return err;
            }
            
            ESP_LOGI(TAG, "Updated alarm: %s (ID: %d)", updated_alarm.note.c_str(), updated_alarm.id);
            return ESP_OK;
        }
    }
    
    ESP_LOGW(TAG, "Alarm with ID %d not found for update", updated_alarm.id);
    return ESP_ERR_NOT_FOUND;
}

void AlarmManager::SetAlarmCallback(std::function<void(const Alarm&)> callback) {
    alarm_callback_ = callback;
}

esp_err_t AlarmManager::EnableAlarm(int id, bool enable) {
    for (auto& alarm : alarms_) {
        if (alarm.id == id) {
            alarm.enabled = enable;
            
            // 保存到NVS
            esp_err_t err = SaveAlarmsToNVS();
            if (err != ESP_OK) {
                return err;
            }
            
            ESP_LOGI(TAG, "%s alarm with ID: %d", enable ? "Enabled" : "Disabled", id);
            return ESP_OK;
        }
    }
    
    ESP_LOGW(TAG, "Alarm with ID %d not found", id);
    return ESP_ERR_NOT_FOUND;
}

Alarm* AlarmManager::GetNearestAlarm() {
    if (alarms_.empty()) {
        return nullptr;
    }
    
    time_t now = time(nullptr);
    tm current_time = {0};
    localtime_r(&now, &current_time);
    
    Alarm* nearest_alarm = nullptr;
    time_t nearest_time = 0;
    
    for (auto& alarm : alarms_) {
        // 检查闹钟是否启用
        if (!alarm.enabled) {
            continue;
        }
        
        // 检查闹钟数据是否有效
        if (alarm.note.empty()) {
            ESP_LOGW(TAG, "闹钟备注为空，跳过检查: ID=%d", alarm.id);
            continue;
        }
        
        time_t next_time = GetNextAlarmTime(alarm);
        if (next_time > now) {
            if (nearest_alarm == nullptr || next_time < nearest_time) {
                nearest_alarm = &alarm;
                nearest_time = next_time;
            }
        }
    }
    
    return nearest_alarm;
}

int64_t AlarmManager::GetNextAlarmWakeupTimeUs() const {
    const Alarm* nearest_alarm = nullptr;
    time_t nearest_time = 0;
    
    time_t now = time(nullptr);
    tm current_time = {0};
    localtime_r(&now, &current_time);
    
    // 打印当前时间
    ESP_LOGI(TAG, "计算下一个闹钟唤醒时间 - 当前时间: %04d-%02d-%02d %02d:%02d:%02d", 
             current_time.tm_year + 1900, current_time.tm_mon + 1, current_time.tm_mday,
             current_time.tm_hour, current_time.tm_min, current_time.tm_sec);
    
    for (const auto& alarm : alarms_) {
        // 检查闹钟是否启用
        if (!alarm.enabled) {
            continue;
        }
        
        // 检查闹钟数据是否有效
        if (alarm.note.empty()) {
            ESP_LOGW(TAG, "闹钟备注为空，跳过检查: ID=%d", alarm.id);
            continue;
        }
        
        time_t next_time = GetNextAlarmTime(alarm);
        if (next_time > now) {
            // 打印找到的闹钟
            tm next_tm = {0};
            localtime_r(&next_time, &next_tm);
            ESP_LOGI(TAG, "找到有效闹钟: ID=%d, 下次触发时间=%04d-%02d-%02d %02d:%02d:%02d", 
                     alarm.id, next_tm.tm_year + 1900, next_tm.tm_mon + 1, next_tm.tm_mday,
                     next_tm.tm_hour, next_tm.tm_min, next_tm.tm_sec);
            
            if (nearest_alarm == nullptr || next_time < nearest_time) {
                nearest_alarm = &alarm;
                nearest_time = next_time;
            }
        }
    }
    
    if (nearest_alarm != nullptr) {
        // 返回微秒数
        int64_t wakeup_time_us = (int64_t)(nearest_time - now) * 1000000;
        tm nearest_tm = {0};
        localtime_r(&nearest_time, &nearest_tm);
        ESP_LOGI(TAG, "最近的闹钟: ID=%d, 触发时间=%04d-%02d-%02d %02d:%02d:%02d, 唤醒时间=%lld 微秒", 
                 nearest_alarm->id, nearest_tm.tm_year + 1900, nearest_tm.tm_mon + 1, nearest_tm.tm_mday,
                 nearest_tm.tm_hour, nearest_tm.tm_min, nearest_tm.tm_sec, (long long)wakeup_time_us);
        return wakeup_time_us;
    }
    
    // 如果没有找到闹钟，返回-1表示不设置定时器唤醒
    ESP_LOGI(TAG, "没有找到有效的闹钟，返回-1");
    return -1;
}

void AlarmManager::TriggerAlarmCheck() {
    // 只有在有闹钟的情况下才执行检查
    if (!alarms_.empty()) {
        CheckAlarms();
    } else {
        ESP_LOGI(TAG, "没有闹钟，跳过闹钟检查");
    }
}
