#pragma once

#include <driver/i2c_master.h>
#include <ctime>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "pcf8563_regs.h"

class PCF8563 {
public:
    // CLKOUT 频率
    enum class ClkOutFreq : uint8_t {
        DISABLED = 0,
        FREQ_32768HZ,
        FREQ_1024HZ,
        FREQ_32HZ,
        FREQ_1HZ,
    };

    // Timer 频率
    enum class TimerClock : uint8_t {
        FREQ_4096HZ = 0,
        FREQ_64HZ,
        FREQ_1HZ,
        FREQ_1_60HZ
    };

    // Alarm 类型
    enum AlarmFlag : uint8_t {
        MATCH_MIN     = 0x01,
        MATCH_HOUR    = 0x02,
        MATCH_DAY     = 0x04,
        MATCH_WEEKDAY = 0x08
    };

    // 构造函数
    PCF8563(uint8_t device_address = 0x51);
    
    // 初始化函数
    esp_err_t init(i2c_master_bus_handle_t i2c_bus);

    // 析构函数
    ~PCF8563();

    // 时间操作
    esp_err_t setTime(const struct tm &time);
    esp_err_t getTime(struct tm &time, bool &valid);
    esp_err_t startClock();
    esp_err_t stopClock();
    bool isClockRunning();

    // CLKOUT
    esp_err_t setClkOut(ClkOutFreq freq);
    esp_err_t getClkOut(ClkOutFreq &freq);

    // Timer
    esp_err_t setTimerSettings(bool int_enable, TimerClock clock);
    esp_err_t getTimerSettings(bool &int_enabled, TimerClock &clock);
    esp_err_t setTimerValue(uint8_t value);
    esp_err_t getTimerValue(uint8_t &value);
    esp_err_t startTimer();
    esp_err_t stopTimer();
    esp_err_t getTimerFlag(bool &flag);
    esp_err_t clearTimerFlag();

    // Alarm
    esp_err_t setAlarm(bool int_enable, uint8_t flags, const struct tm &time);
    esp_err_t getAlarm(bool &int_enabled, uint8_t &flags, struct tm &time);
    esp_err_t getAlarmFlag(bool &flag);
    esp_err_t clearAlarmFlag();

private:
    i2c_master_dev_handle_t device_handle_;
    SemaphoreHandle_t mutex_;

    static constexpr uint8_t PCF8563_I2C_ADDR = 0x51;

    static uint8_t dec2bcd(uint8_t val) { return ((val / 10) << 4) + (val % 10); }
    static uint8_t bcd2dec(uint8_t val) { return ((val >> 4) * 10 + (val & 0x0F)); }
    static constexpr uint8_t BV(uint8_t x) { return (1 << x); }

    // 内部读写封装
    esp_err_t writeReg(uint8_t reg, uint8_t val);
    esp_err_t readReg(uint8_t reg, uint8_t &val);
    esp_err_t updateReg(uint8_t reg, uint8_t mask, uint8_t val);
};
