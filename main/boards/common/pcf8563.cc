#include "pcf8563.h"
#include <cstring>
#include <esp_log.h>
#include "pcf8563_regs.h"

#define I2C_TIMEOUT_MS 1000

PCF8563::PCF8563(uint8_t device_address)
{
    mutex_ = xSemaphoreCreateMutex();
    device_handle_ = NULL;  // 初始化为NULL
}

esp_err_t PCF8563::init(i2c_master_bus_handle_t i2c_bus)
{
    // 创建I2C设备句柄
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = PCF8563_I2C_ADDR;  // 使用默认地址
    dev_cfg.scl_speed_hz = 100000; // Standard mode
    
    esp_err_t err = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &device_handle_);
    if (err != ESP_OK) {
        ESP_LOGE("PCF8563", "添加PCF8563 I2C设备失败: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI("PCF8563", "PCF8563设备初始化完成，地址: 0x%02x", PCF8563_I2C_ADDR);
    
    // 读取初始状态寄存器值
    uint8_t ctrl_status1, ctrl_status2;
    if (readReg(REG_CTRL_STATUS1, ctrl_status1) == ESP_OK) {
        ESP_LOGI("PCF8563", "初始CTRL_STATUS1值: 0x%02x", ctrl_status1);
    }
    if (readReg(REG_CTRL_STATUS2, ctrl_status2) == ESP_OK) {
        ESP_LOGI("PCF8563", "初始CTRL_STATUS2值: 0x%02x", ctrl_status2);
    }
    
    // 检查电源是否正常
    struct tm time_info;
    bool valid;
    if (getTime(time_info, valid) == ESP_OK) {
        if (valid) {
            ESP_LOGI("PCF8563", "RTC初始时间: %04d-%02d-%02d %02d:%02d:%02d", 
                     time_info.tm_year + 1900, time_info.tm_mon + 1, time_info.tm_mday,
                     time_info.tm_hour, time_info.tm_min, time_info.tm_sec);
        } else {
            ESP_LOGW("PCF8563", "RTC电源失效(VL位被设置)，时间数据无效");
        }
    }
    
    // 初始化时启动RTC时钟
    err = startClock();
    if (err != ESP_OK) {
        ESP_LOGW("PCF8563", "启动RTC时钟失败: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("PCF8563", "RTC时钟已启动");
        // 验证时钟是否真正开始运行
        bool clock_running = isClockRunning();
        ESP_LOGI("PCF8563", "时钟运行状态: %s", clock_running ? "运行中" : "已停止");
        
        // 如果时钟未运行，尝试多次启动
        if (!clock_running) {
            ESP_LOGW("PCF8563", "时钟未运行，尝试重新启动");
            for (int i = 0; i < 3; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                err = startClock();
                if (err == ESP_OK) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                    clock_running = isClockRunning();
                    if (clock_running) {
                        ESP_LOGI("PCF8563", "第%d次尝试后时钟成功启动", i+1);
                        break;
                    }
                }
            }
            if (!clock_running) {
                ESP_LOGE("PCF8563", "多次尝试后时钟仍未能启动");
            }
        }
    }
    
    return ESP_OK;
}

PCF8563::~PCF8563()
{
    i2c_master_bus_rm_device(device_handle_);
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}



esp_err_t PCF8563::writeReg(uint8_t reg, uint8_t val)
{
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    uint8_t data[2] = {reg, val};
    esp_err_t ret = i2c_master_transmit(device_handle_, data, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    
    xSemaphoreGive(mutex_);
    return ret;
}

esp_err_t PCF8563::readReg(uint8_t reg, uint8_t &val)
{
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    esp_err_t ret = i2c_master_transmit_receive(device_handle_, &reg, 1, &val, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    
    xSemaphoreGive(mutex_);
    return ret;
}

esp_err_t PCF8563::updateReg(uint8_t reg, uint8_t mask, uint8_t val)
{
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    uint8_t current_val;
    esp_err_t ret = i2c_master_transmit_receive(device_handle_, &reg, 1, &current_val, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        xSemaphoreGive(mutex_);
        return ret;
    }
    
    uint8_t new_val = (current_val & ~mask) | val;
    if (new_val != current_val) {
        uint8_t data[2] = {reg, new_val};
        ret = i2c_master_transmit(device_handle_, data, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    }
    
    xSemaphoreGive(mutex_);
    return ret;
}

// 设置时间
esp_err_t PCF8563::setTime(const struct tm& time)
{
    bool ovf = time.tm_year >= 200;

    uint8_t data[7] = {
        dec2bcd(static_cast<uint8_t>(time.tm_sec)),
        dec2bcd(static_cast<uint8_t>(time.tm_min)),
        dec2bcd(static_cast<uint8_t>(time.tm_hour)),
        dec2bcd(static_cast<uint8_t>(time.tm_mday)),
        dec2bcd(static_cast<uint8_t>(time.tm_wday)),
        static_cast<uint8_t>(dec2bcd(static_cast<uint8_t>(time.tm_mon + 1)) | (ovf ? BV(7) : 0)),
        dec2bcd(static_cast<uint8_t>(time.tm_year - (ovf ? 200 : 100)))
    };

    // 添加调试信息
    ESP_LOGI("PCF8563", "准备写入时间: %02d:%02d:%02d", time.tm_hour, time.tm_min, time.tm_sec);
    ESP_LOGD("PCF8563", "写入数据: SEC=0x%02x, MIN=0x%02x, HOUR=0x%02x, DAY=0x%02x, WEEKDAY=0x%02x, MONTH=0x%02x, YEAR=0x%02x", 
             data[0], data[1], data[2], data[3], data[4], data[5], data[6]);

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // 构造要写入的数据，包括寄存器地址和时间数据
    uint8_t write_data[8] = {REG_VL_SECONDS, data[0], data[1], data[2], data[3], data[4], data[5], data[6]};
    esp_err_t ret = i2c_master_transmit(device_handle_, write_data, 8, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    
    xSemaphoreGive(mutex_);
    
    if (ret == ESP_OK) {
        ESP_LOGI("PCF8563", "时间写入成功");
        // 写入成功后，验证写入的数据
        struct tm verify_time;
        bool valid;
        if (getTime(verify_time, valid) == ESP_OK && valid) {
            ESP_LOGI("PCF8563", "验证读取时间: %02d:%02d:%02d", verify_time.tm_hour, verify_time.tm_min, verify_time.tm_sec);
        }
    } else {
        ESP_LOGE("PCF8563", "时间写入失败: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// 获取时间
esp_err_t PCF8563::getTime(struct tm& time, bool& valid)
{
    uint8_t data[7];
    
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // 先发送寄存器地址
    uint8_t reg = REG_VL_SECONDS; // REG_VL_SECONDS
    esp_err_t ret = i2c_master_transmit_receive(device_handle_, &reg, 1, data, 7, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    xSemaphoreGive(mutex_);
    
    if (ret != ESP_OK) {
        ESP_LOGE("PCF8563", "读取时间数据失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 添加调试信息
    ESP_LOGD("PCF8563", "读取数据: SEC=0x%02x, MIN=0x%02x, HOUR=0x%02x, DAY=0x%02x, WEEKDAY=0x%02x, MONTH=0x%02x, YEAR=0x%02x", 
             data[0], data[1], data[2], data[3], data[4], data[5], data[6]);

    valid = !(data[0] & BV(BIT_VL));
    if (!valid) {
        ESP_LOGW("PCF8563", "RTC数据无效(VL位被设置)");
    }
    
    time.tm_sec  = static_cast<int>(bcd2dec(static_cast<uint8_t>(data[0] & ~BV(BIT_VL))));
    time.tm_min  = static_cast<int>(bcd2dec(static_cast<uint8_t>(data[1] & MASK_MIN)));
    time.tm_hour = static_cast<int>(bcd2dec(static_cast<uint8_t>(data[2] & MASK_HOUR)));
    time.tm_mday = static_cast<int>(bcd2dec(static_cast<uint8_t>(data[3] & MASK_MDAY)));
    time.tm_wday = static_cast<int>(bcd2dec(static_cast<uint8_t>(data[4] & MASK_WDAY)));
    time.tm_mon  = static_cast<int>(bcd2dec(static_cast<uint8_t>(data[5] & MASK_MON)) - 1);
    time.tm_year = static_cast<int>(bcd2dec(static_cast<uint8_t>(data[6])) + (data[5] & BV(BIT_VL) ? 200 : 100));
    
    // 添加调试信息
    ESP_LOGD("PCF8563", "解析后时间: %04d-%02d-%02d %02d:%02d:%02d", 
             time.tm_year + 1900, time.tm_mon + 1, time.tm_mday, 
             time.tm_hour, time.tm_min, time.tm_sec);

    return ESP_OK;
}

// 启动RTC时钟
esp_err_t PCF8563::startClock()
{
    // 在清除STOP位之前，先读取当前寄存器值
    uint8_t current_val;
    esp_err_t ret = readReg(REG_CTRL_STATUS1, current_val);
    if (ret != ESP_OK) {
        ESP_LOGE("PCF8563", "读取CTRL_STATUS1寄存器失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI("PCF8563", "CTRL_STATUS1当前值: 0x%02x", current_val);
    ESP_LOGI("PCF8563", "STOP位状态: %s", (current_val & BV(BIT_STOP)) ? "设置" : "未设置");
    
    // 检查其他可能影响时钟运行的位
    if (current_val & 0x80) {  // 测试位
        ESP_LOGW("PCF8563", "CTRL_STATUS1寄存器中的测试位被设置");
    }
    
    // 清除控制/状态寄存器1的STOP位（第5位）
    ret = updateReg(REG_CTRL_STATUS1, BV(BIT_STOP), 0);
    
    if (ret == ESP_OK) {
        // 启动后再次读取寄存器值验证
        uint8_t new_val;
        if (readReg(REG_CTRL_STATUS1, new_val) == ESP_OK) {
            ESP_LOGI("PCF8563", "CTRL_STATUS1更新后值: 0x%02x", new_val);
            bool is_running = !(new_val & BV(BIT_STOP));
            ESP_LOGI("PCF8563", "时钟状态验证: %s", is_running ? "运行中" : "已停止");
            
            // 检查STOP位是否真的被清除了
            if (new_val & BV(BIT_STOP)) {
                ESP_LOGE("PCF8563", "STOP位未能成功清除");
            }
        }
        
        // 等待一小段时间后再次验证
        vTaskDelay(pdMS_TO_TICKS(100));
        bool actually_running = isClockRunning();
        ESP_LOGI("PCF8563", "时钟实际运行状态: %s", actually_running ? "运行中" : "已停止");
    }
    
    return ret;
}

// 检查RTC时钟是否运行
bool PCF8563::isClockRunning()
{
    uint8_t val;
    esp_err_t err = readReg(REG_CTRL_STATUS1, val); // 读取控制/状态寄存器1
    if (err != ESP_OK) {
        ESP_LOGE("PCF8563", "读取CTRL_STATUS1寄存器失败: %s", esp_err_to_name(err));
        return false;
    }
    
    ESP_LOGD("PCF8563", "CTRL_STATUS1寄存器值: 0x%02x", val);
    
    // 如果STOP位（第5位）为0，则时钟运行
    bool running = !(val & BV(BIT_STOP));
    
    // 如果时钟报告为运行状态，进一步验证时钟是否真的在走动
    if (running) {
        // 读取当前时间
        struct tm time1, time2;
        bool valid1, valid2;
        if (getTime(time1, valid1) == ESP_OK && valid1) {
            // 等待一小段时间
            vTaskDelay(pdMS_TO_TICKS(1000));
            // 再次读取时间
            if (getTime(time2, valid2) == ESP_OK && valid2) {
                // 检查秒数是否变化
                if (time1.tm_sec != time2.tm_sec) {
                    ESP_LOGD("PCF8563", "时钟验证通过：时间在走动 (%02d:%02d:%02d -> %02d:%02d:%02d)",
                             time1.tm_hour, time1.tm_min, time1.tm_sec,
                             time2.tm_hour, time2.tm_min, time2.tm_sec);
                    return true;
                } else {
                    ESP_LOGW("PCF8563", "时钟报告为运行但时间未变化，可能存在故障 (%02d:%02d:%02d)",
                             time1.tm_hour, time1.tm_min, time1.tm_sec);
                    return false;
                }
            }
        }
    }
    
    return running;
}

// 停止RTC时钟
esp_err_t PCF8563::stopClock()
{
    // 设置控制/状态寄存器1的STOP位（第5位）
    return updateReg(REG_CTRL_STATUS1, BV(BIT_STOP), BV(BIT_STOP));
}

// 设置 CLKOUT
esp_err_t PCF8563::setClkOut(ClkOutFreq freq)
{
    uint8_t val = (freq == ClkOutFreq::DISABLED) ? 0 : (BV(BIT_CLKOUT_FE) | (static_cast<uint8_t>(freq) - 1));
    return writeReg(REG_CLKOUT, val); // CLKOUT寄存器
}

esp_err_t PCF8563::getClkOut(ClkOutFreq &freq)
{
    uint8_t val;
    esp_err_t ret = readReg(REG_CLKOUT, val); // CLKOUT寄存器
    if (ret != ESP_OK) return ret;
    
    if (val & BV(BIT_CLKOUT_FE)) {
        freq = static_cast<ClkOutFreq>((val & 3) + 1);
    } else {
        freq = ClkOutFreq::DISABLED;
    }
    
    return ESP_OK;
}

// Timer相关函数
esp_err_t PCF8563::setTimerSettings(bool int_enable, TimerClock clock)
{
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // 更新CTRL_STATUS2寄存器
    uint8_t ctrl_status2;
    uint8_t reg1 = REG_CTRL_STATUS2;
    esp_err_t ret = i2c_master_transmit_receive(device_handle_, &reg1, 1, &ctrl_status2, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        xSemaphoreGive(mutex_);
        return ret;
    }
    
    if (int_enable) {
        ctrl_status2 |= BV(0); // 设置TIE位
    } else {
        ctrl_status2 &= ~BV(0); // 清除TIE位
    }
    
    uint8_t data1[2] = {reg1, ctrl_status2};
    ret = i2c_master_transmit(device_handle_, data1, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        xSemaphoreGive(mutex_);
        return ret;
    }
    
    // 更新TIMER_CTRL寄存器
    uint8_t timer_ctrl = static_cast<uint8_t>(clock) & 0x03;
    uint8_t timer_data[2] = {REG_TIMER_CTRL, timer_ctrl};
    ret = i2c_master_transmit(device_handle_, timer_data, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    
    xSemaphoreGive(mutex_);
    return ret;
}

esp_err_t PCF8563::getTimerSettings(bool &int_enabled, TimerClock &clock)
{
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    uint8_t ctrl_status2, timer_ctrl;
    uint8_t reg1 = REG_CTRL_STATUS2; // REG_CTRL_STATUS2
    esp_err_t ret = i2c_master_transmit_receive(device_handle_, &reg1, 1, &ctrl_status2, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        xSemaphoreGive(mutex_);
        return ret;
    }
    
    uint8_t timer_reg = REG_TIMER_CTRL; // REG_TIMER_CTRL
    ret = i2c_master_transmit_receive(device_handle_, &timer_reg, 1, &timer_ctrl, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        xSemaphoreGive(mutex_);
        return ret;
    }
    
    int_enabled = (ctrl_status2 & BV(0)) ? true : false;
    clock = static_cast<TimerClock>(timer_ctrl & 0x03);
    
    xSemaphoreGive(mutex_);
    return ESP_OK;
}

esp_err_t PCF8563::setTimerValue(uint8_t value)
{
    return writeReg(REG_TIMER, value); // TIMER寄存器
}

esp_err_t PCF8563::getTimerValue(uint8_t &value)
{
    return readReg(REG_TIMER, value); // TIMER寄存器
}

esp_err_t PCF8563::startTimer()
{
    return updateReg(REG_TIMER_CTRL, BV(BIT_TIMER_CTRL_TE), BV(BIT_TIMER_CTRL_TE)); // TIMER_CTRL寄存器的TE位
}

esp_err_t PCF8563::stopTimer()
{
    return updateReg(REG_TIMER_CTRL, BV(BIT_TIMER_CTRL_TE), 0); // TIMER_CTRL寄存器的TE位
}

esp_err_t PCF8563::getTimerFlag(bool &flag)
{
    uint8_t val;
    esp_err_t ret = readReg(REG_CTRL_STATUS2, val); // CTRL_STATUS2寄存器
    if (ret != ESP_OK) return ret;
    
    flag = (val & BV(3)) ? true : false; // TF位
    return ESP_OK;
}

esp_err_t PCF8563::clearTimerFlag()
{
    return updateReg(REG_CTRL_STATUS2, BV(BIT_CTRL_STATUS2_TF), 0); // CTRL_STATUS2寄存器的TF位
}

// Alarm相关函数
esp_err_t PCF8563::setAlarm(bool int_enable, uint8_t flags, const struct tm &time)
{
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // 更新CTRL_STATUS2寄存器
    uint8_t ctrl_status2;
    uint8_t reg1 = REG_CTRL_STATUS2;
    esp_err_t ret = i2c_master_transmit_receive(device_handle_, &reg1, 1, &ctrl_status2, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        xSemaphoreGive(mutex_);
        return ret;
    }
    
    if (int_enable) {
        ctrl_status2 |= BV(1); // 设置AIE位
    } else {
        ctrl_status2 &= ~BV(1); // 清除AIE位
    }
    
    uint8_t data1[2] = {reg1, ctrl_status2};
    ret = i2c_master_transmit(device_handle_, data1, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        xSemaphoreGive(mutex_);
        return ret;
    }
    
    // 写入闹钟时间
    uint8_t data[4] = {
        static_cast<uint8_t>(dec2bcd(static_cast<uint8_t>(time.tm_min)) | ((flags & MATCH_MIN) ? 0 : BV(7))),
        static_cast<uint8_t>(dec2bcd(static_cast<uint8_t>(time.tm_hour)) | ((flags & MATCH_HOUR) ? 0 : BV(7))),
        static_cast<uint8_t>(dec2bcd(static_cast<uint8_t>(time.tm_mday)) | ((flags & MATCH_DAY) ? 0 : BV(7))),
        static_cast<uint8_t>(dec2bcd(static_cast<uint8_t>(time.tm_wday)) | ((flags & MATCH_WEEKDAY) ? 0 : BV(7)))
    };
    
    uint8_t alarm_reg = REG_ALARM_MIN;
    uint8_t alarm_data[5] = {alarm_reg, data[0], data[1], data[2], data[3]};
    ret = i2c_master_transmit(device_handle_, alarm_data, 5, pdMS_TO_TICKS(I2C_TIMEOUT_MS)); // 从ALARM_MIN寄存器开始
    
    xSemaphoreGive(mutex_);
    return ret;
}

esp_err_t PCF8563::getAlarm(bool &int_enabled, uint8_t &flags, struct tm &time)
{
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    uint8_t ctrl_status2, data[4];
    uint8_t reg1 = REG_CTRL_STATUS2; // REG_CTRL_STATUS2
    esp_err_t ret = i2c_master_transmit_receive(device_handle_, &reg1, 1, &ctrl_status2, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        xSemaphoreGive(mutex_);
        return ret;
    }
    
    uint8_t alarm_reg = REG_ALARM_MIN;
    ret = i2c_master_transmit_receive(device_handle_, &alarm_reg, 1, data, 4, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        xSemaphoreGive(mutex_);
        return ret;
    }
    
    int_enabled = (ctrl_status2 & BV(1)) ? true : false;
    flags = 0;
    
    if (!(data[0] & BV(7)))
        flags |= MATCH_MIN;
    if (!(data[1] & BV(7)))
        flags |= MATCH_HOUR;
    if (!(data[2] & BV(7)))
        flags |= MATCH_DAY;
    if (!(data[3] & BV(7)))
        flags |= MATCH_WEEKDAY;

    time.tm_min = static_cast<int>(bcd2dec(static_cast<uint8_t>(data[0] & 0x7F)));
    time.tm_hour = static_cast<int>(bcd2dec(static_cast<uint8_t>(data[1] & 0x3F)));
    time.tm_mday = static_cast<int>(bcd2dec(static_cast<uint8_t>(data[2] & 0x3F)));
    time.tm_wday = static_cast<int>(bcd2dec(static_cast<uint8_t>(data[3] & 0x07)));
    
    xSemaphoreGive(mutex_);
    return ESP_OK;
}

esp_err_t PCF8563::getAlarmFlag(bool &flag)
{
    uint8_t val;
    esp_err_t ret = readReg(REG_CTRL_STATUS2, val); // CTRL_STATUS2寄存器
    if (ret != ESP_OK) return ret;
    
    flag = (val & BV(4)) ? true : false; // AF位
    return ESP_OK;
}

esp_err_t PCF8563::clearAlarmFlag()
{
    return updateReg(REG_CTRL_STATUS2, BV(BIT_CTRL_STATUS2_AF), 0); // CTRL_STATUS2寄存器的AF位
}
