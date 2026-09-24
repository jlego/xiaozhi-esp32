#include "wifi_board.h"
#include "codecs/es8311_audio_codec.h"
#include "display/lcd_display.h"
#include "display/emote_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "sleep_timer.h"
#include "power_manager.h"
#include "pcf8563.h"
#include "alarm.h"
// #include "assets/lang_config.h"  // 添加语言配置头文件

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <wifi_manager.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>
#include <esp_lcd_touch_cst816s.h>
#include "esp_lcd_jd9853.h"
#include <driver/gpio.h>

#define TAG "EspWatchS3b3Board"

// 控制函数
void ControlVibrationMotor(bool enable) {
    gpio_set_level(VIB_MOTOR_PIN, enable ? 1 : 0);
}

// 初始化振动马达GPIO
void InitializeVibrationMotor() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << VIB_MOTOR_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(VIB_MOTOR_PIN, 0); // 初始状态关闭
}

// 闹钟触发任务的辅助函数
void AlarmTask(void* pvParameters) {
    Alarm* alarm = static_cast<Alarm*>(pvParameters);
    // 检查闹钟数据是否有效
    if (alarm == nullptr) {
        ESP_LOGE(TAG, "闹钟数据为空，无法执行闹钟任务");
        vTaskDelete(NULL);
        return;
    }
    // 检查闹钟备注是否有效
    if (alarm->note.empty()) {
        ESP_LOGW(TAG, "闹钟备注为空，使用默认文本");
        // 可以设置一个默认的备注文本
        // alarm->note = "闹钟";
    }
    ESP_LOGI(TAG, "闹钟任务开始执行，备注: %s", alarm->note.c_str());
    // 添加延迟以确保系统完全从浅睡眠中唤醒并且硬件稳定
    vTaskDelay(pdMS_TO_TICKS(150));
    // 显示闹钟备注文本在提示框中
    auto& board = Board::GetInstance();
    auto* display = board.GetDisplay();
    if (display != nullptr) {
        ESP_LOGI(TAG, "调用ShowAlarmNotification显示闹钟提示框");
        display->ShowAlarmNotification(alarm->note.c_str());
    } else {
        ESP_LOGW(TAG, "显示设备为空");
    }
    // 添加一个小延迟确保显示操作完成
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // 循环振动5次
    for (int i = 0; i < 5; i++) {
        // 开启振动
        ControlVibrationMotor(true);
        // 振动1秒
        vTaskDelay(pdMS_TO_TICKS(1000));
        // 停止振动
        ControlVibrationMotor(false);
        // 间隔1秒再重复
        if (i < 4) { // 最后一次不需要等待
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    // 隐藏闹钟提示框
    ESP_LOGI(TAG, "准备隐藏闹钟提示框");
    if (display != nullptr) {
        display->HideAlarmNotification();
    }
    // 添加一个小延迟确保隐藏操作完成
    vTaskDelay(pdMS_TO_TICKS(20));
    // 清理闹钟数据副本
    delete alarm;
    ESP_LOGI(TAG, "闹钟任务执行完毕");
    vTaskDelete(NULL); // 删除任务自身
}

void OnAlarmTriggered(const Alarm& alarm) {
    ESP_LOGI(TAG, "闹钟触发: %s (ID: %d, 时间: %02d:%02d, 重复模式: %d)", 
             alarm.note.c_str(), alarm.id, alarm.hour, alarm.minute, static_cast<int>(alarm.repeat));
    // 检查闹钟数据是否有效
    if (alarm.note.empty()) {
        ESP_LOGW(TAG, "闹钟备注为空，跳过闹钟触发");
        return;
    }
    // 在实际应用中，这里可以添加更多处理逻辑
    // 例如播放音乐、震动提醒等
    // 由于xTaskCreate需要普通函数指针，我们将闹钟数据传递给任务参数
    Alarm* alarm_copy = new Alarm(alarm); // 创建闹钟数据的副本
    xTaskCreate(AlarmTask, "alarm_sound", 4096, alarm_copy, 5, NULL);
}

class EspWatchS3b3Board : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    LcdDisplay* display_;
    SleepTimer* power_save_timer_ = nullptr;
    PowerManager* power_manager_;
    AlarmManager alarm_manager;
    PCF8563* rtc_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;

public:
    // 实现 WifiBoard 的虚函数
    virtual int64_t GetNextAlarmWakeupTimeUs() const override {
        // 使用闹钟管理器获取下一个闹钟的唤醒时间
        return alarm_manager.GetNextAlarmWakeupTimeUs();
    }
    
    virtual Alarm* GetNearestAlarm() override {
        // 获取最近的闹钟
        Alarm* nearest = alarm_manager.GetNearestAlarm();
        if (nearest) {
            ESP_LOGI(TAG, "最近的闹钟: %s (ID: %d, 时间: %02d:%02d)", 
                    nearest->note.c_str(), nearest->id, nearest->hour, nearest->minute);
        } else {
            ESP_LOGI(TAG, "没有即将到来的闹钟");
        }
        return nearest;
    }
    
    virtual void TriggerAlarmCheck() override {
        // 触发闹钟检查
        alarm_manager.TriggerAlarmCheck();
    }

private:
    // 新增安全重启音频编解码器的方法
    void SafeRestartAudioCodec() {
        auto* codec = GetAudioCodec();
        if (codec != nullptr) {
            // 在重新启动之前，先确保关闭任何现有的I2S通道
            // 这是为了避免ESP_ERR_INVALID_STATE错误
            // 临时禁用输入输出
            codec->EnableInput(false);
            codec->EnableOutput(false);
            // 添加小延迟确保状态变更完成
            vTaskDelay(pdMS_TO_TICKS(10));
            // 重新启用输入输出
            codec->EnableInput(true);
            codec->EnableOutput(true);
        }
    }

    void InitializeAlarmManager() {
        // 注册闹钟触发回调函数
        alarm_manager.SetAlarmCallback(OnAlarmTriggered);
        // 初始化闹钟管理器（这会加载已保存的闹钟数据）
        alarm_manager.Init();
    }

    void InitializePowerManager() {
        power_manager_ = new PowerManager(i2c_bus_);
        power_manager_->OnChargingStatusChanged([this](bool is_charging) {
            if (is_charging) {
                power_save_timer_->SetEnabled(false);
            } else {
                power_save_timer_->SetEnabled(true);
            }
        });
    }

    void InitializePowerSaveTimer() {
        power_save_timer_ = new SleepTimer(10);
        power_save_timer_->OnEnterLightSleepMode([this]() {
            ESP_LOGI(TAG, "Enabling sleep mode");
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(0);
            
        });
        power_save_timer_->OnExitLightSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->SetBrightness(5);
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeBm8563() {
        // 创建PCF8563对象
        rtc_ = new PCF8563();
        esp_err_t err = rtc_->init(i2c_bus_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "PCF8563 initialization failed: %s", esp_err_to_name(err));
            // 不要崩溃，继续运行，只是RTC功能不可用
        }
    }

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }
    
    void InitializeSpi() {
        // 初始化SPI总线
        ESP_LOGI(TAG, "Initializing SPI3_HOST bus...");
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_SCLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            // 更新emoji box中的时间显示
            auto display = Board::GetInstance().GetDisplay();
            display->UpdateEmojiBoxTime();
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiManager::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            if(app.GetDeviceState() == kDeviceStateIdle && GetBacklight()->brightness() == 0){
                GetBacklight()->SetBrightness(5);
                // 这样可以确保背光只在所有显示操作完成后才点亮
                ESP_LOGI(TAG, "Device waking up from sleep mode");
            }else{
                app.ToggleChatState();
            }
        });

#if CONFIG_USE_DEVICE_AEC
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
            }
        });
#endif
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片ST7789
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RESET_PIN;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel_));
        
        esp_lcd_panel_reset(panel_);
        esp_lcd_panel_init(panel_);
        esp_lcd_panel_invert_color(panel_, true);
        esp_lcd_panel_swap_xy(panel_, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel_, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        display_ = new emote::EmoteDisplay(panel_, panel_io, DISPLAY_WIDTH, DISPLAY_HEIGHT);
#else
        display_ = new SpiLcdDisplay(panel_io, panel_,
            DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
#endif
    }

    void InitializeJd9853Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 20 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片JD9853
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_jd9853(panel_io, &panel_config, &panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, DISPLAY_INVERT_COLOR));
        ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_, 0, 0));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_, true, false));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_, true));
#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        display_ = new emote::EmoteDisplay(panel_, panel_io, DISPLAY_WIDTH, DISPLAY_HEIGHT);
#else
        display_ = new SpiLcdDisplay(panel_io, panel_,
            DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
#endif
    }

    void InitializeTouch() {
        esp_lcd_touch_handle_t tp;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH - 1,
            .y_max = DISPLAY_HEIGHT - 1,
            .rst_gpio_num = GPIO_NUM_10,
            .int_gpio_num = GPIO_NUM_11,
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 0,
                .mirror_x = 0,
                .mirror_y = 0,
            },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = {
            .dev_addr = ESP_LCD_TOUCH_IO_I2C_CST816S_ADDRESS,
            .scl_speed_hz = 400 * 1000,
            .control_phase_bytes = 1,
            .dc_bit_offset = 0,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 0,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 1,
            },
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle));
        ESP_LOGI(TAG, "Initialize touch controller");
        ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp));
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(),
            .handle = tp,
        };
        lvgl_port_add_touch(&touch_cfg);
        ESP_LOGI(TAG, "Touch panel initialized successfully");
    }

public:
    EspWatchS3b3Board() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializePowerSaveTimer();
        InitializePowerManager();
        InitializeBm8563();
        InitializeSpi();
        #ifdef LCD_TYPE_JD9853_SERIAL
        InitializeJd9853Display(); 
        #else
        InitializeSt7789Display(); 
        #endif
        InitializeButtons();
        // InitializeTouch();
        InitializeAlarmManager();
        InitializeVibrationMotor(); // 初始化振动马达
        GetBacklight()->SetBrightness(5);

#if CONFIG_IOT_PROTOCOL_XIAOZHI
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
#endif
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(
            i2c_bus_, 
            I2C_NUM_0, 
            AUDIO_INPUT_SAMPLE_RATE, 
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, 
            AUDIO_I2S_GPIO_BCLK, 
            AUDIO_I2S_GPIO_WS, 
            AUDIO_I2S_GPIO_DOUT, 
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, 
            AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        charging = power_manager_->IsCharging();
        discharging = power_manager_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }
        level = power_manager_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveMode(bool enabled) override {
        if (!enabled) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveMode(enabled);
    }

    virtual SleepTimer* GetPowerSaveTimer() override {
        return power_save_timer_;
    }
};

DECLARE_BOARD(EspWatchS3b3Board);