#include "dual_network_board.h"
#include "codecs/es8311_audio_codec.h"
#include "display/lcd_display.h"
#include "display/emote_display.h"
#include "esp_lcd_sh8601.h"
#include "application.h"
#include "mcp_server.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "sleep_timer.h"
#include "power_manager.h"
#include "alarm.h"
#include "assets/lang_config.h"  // 添加语言配置头文件

#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>
#include <esp_lcd_touch_cst9217.h>
#include <esp_err.h>

#define TAG "EspWatchAmoled4gBoard"

#define LCD_OPCODE_WRITE_CMD (0x02ULL)
#define LCD_OPCODE_READ_CMD (0x03ULL)
#define LCD_OPCODE_WRITE_COLOR (0x32ULL)

static const sh8601_lcd_init_cmd_t vendor_specific_init[] = {
    // set display to qspi mode
    {0x11, (uint8_t []){0x00}, 0, 120},
    {0xC4, (uint8_t []){0x80}, 1, 0},
    {0x44, (uint8_t []){0x01, 0xD1}, 2, 0},
    {0x35, (uint8_t []){0x00}, 1, 0},
    {0x53, (uint8_t []){0x20}, 1, 10},
    {0x63, (uint8_t []){0xFF}, 1, 10},
    {0x51, (uint8_t []){0x00}, 1, 10},
    {0x2A, (uint8_t []){0x00,0x16,0x01,0xAF}, 4, 0},
    {0x2B, (uint8_t []){0x00,0x00,0x01,0xF5}, 4, 0},
    {0x29, (uint8_t []){0x00}, 0, 10},
    {0x51, (uint8_t []){0xFF}, 1, 0},
};

// 在waveshare_amoled_2_06类之前添加新的显示类
class AmoledDisplay : public SpiLcdDisplay {
public:
    static void rounder_event_cb(lv_event_t* e) {
        lv_area_t* area = (lv_area_t* )lv_event_get_param(e);
        uint16_t x1 = area->x1;
        uint16_t x2 = area->x2;
        uint16_t y1 = area->y1;
        uint16_t y2 = area->y2;
        // round the start of coordinate down to the nearest 2M number
        area->x1 = (x1 >> 1) << 1;
        area->y1 = (y1 >> 1) << 1;
        // round the end of coordinate up to the nearest 2N+1 number
        area->x2 = ((x2 >> 1) << 1) + 1;
        area->y2 = ((y2 >> 1) << 1) + 1;
    }
    AmoledDisplay(esp_lcd_panel_io_handle_t io_handle,
                     esp_lcd_panel_handle_t panel_handle,
                     int width,
                     int height,
                     int offset_x,
                     int offset_y,
                     bool mirror_x,
                     bool mirror_y,
                     bool swap_xy)
        : SpiLcdDisplay(io_handle, panel_handle, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {
        lv_display_add_event_cb(display_, rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);
    }

    void SetupUI() override {
        LcdDisplay::SetupUI();
        DisplayLockGuard lock(this);
        lv_obj_set_style_pad_left(status_bar_, LV_HOR_RES * 0.1, 0);
        lv_obj_set_style_pad_right(status_bar_, LV_HOR_RES * 0.1, 0);
    }
};

class CustomBacklight : public Backlight {
public:
    CustomBacklight(esp_lcd_panel_io_handle_t panel_io) : Backlight(), panel_io_(panel_io) {}

protected:
    esp_lcd_panel_io_handle_t panel_io_;

    virtual void SetBrightnessImpl(uint8_t brightness) override {
        auto display = Board::GetInstance().GetDisplay();
        DisplayLockGuard lock(display);
        uint8_t data[1] = {((uint8_t)((255*  brightness) / 100))};
        int lcd_cmd = 0x51;
        lcd_cmd &= 0xff;
        lcd_cmd <<= 8;
        lcd_cmd |= LCD_OPCODE_WRITE_CMD << 24;
        esp_lcd_panel_io_tx_param(panel_io_, lcd_cmd, &data, sizeof(data));
    }
};

// 闹钟触发任务的辅助函数
void AlarmTask(void* pvParameters) {
    Alarm* alarm = static_cast<Alarm*>(pvParameters);
    auto& app = Application::GetInstance();
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
    // 确保音频系统正确初始化
    auto& audio_service = app.GetAudioService();
    auto* codec = board.GetAudioCodec();
    if (codec != nullptr) {
        // 安全地启动音频编解码器，避免重复初始化导致的错误
        codec->Start();
        // 临时增加闹钟音量（保存原始音量以便恢复）
        // int original_volume = codec->output_volume();
        codec->SetOutputVolume(100); // 设置最大音量
    }
    // 循环播放音频5次
    for (int i = 0; i < 5; i++) {
        // 开始播放音频
        audio_service.PlaySound(Lang::Sounds::OGG_VIBRATION);
        // 等待一小段时间再重复播放
        vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒重复一次
    }
    // 恢复原始音量
    if (codec != nullptr && codec->output_enabled()) {
        codec->SetOutputVolume(70); // 恢复到默认音量
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

class EspWatchAmoled4gBoard : public DualNetworkBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    AmoledDisplay* display_;
    SleepTimer* power_save_timer_ = nullptr;
    PowerManager* power_manager_;
    AlarmManager alarm_manager;
    CustomBacklight* backlight_;
    esp_lcd_panel_handle_t panel = nullptr;

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
            // 添加小延迟确保音频系统稳定
            vTaskDelay(pdMS_TO_TICKS(20));
            // 清理可能积压的音频数据，避免AFE缓冲区溢出
            // 通过重启音频服务来清理缓冲区
            if (codec->input_enabled()) {
                // 临时禁用并重新启用音频输入以清除缓冲区
                codec->EnableInput(false);
                vTaskDelay(pdMS_TO_TICKS(10));
                codec->EnableInput(true);
            }
        }
    }

    void InitializeAlarmManager() {
        // 注册闹钟触发回调函数
        alarm_manager.SetAlarmCallback(OnAlarmTriggered);
        // 初始化闹钟管理器（这会加载已保存的闹钟数据）
        alarm_manager.Init();
    }

    void InitializePowerSaveTimer() {
        power_save_timer_ = new SleepTimer(10);
        power_save_timer_->OnEnterLightSleepMode([this]() {
            ESP_LOGI(TAG, "Enabling sleep mode");
            // 在进入睡眠前暂停LVGL任务
            if (lv_disp_get_default() != NULL) {
                ESP_LOGI(TAG, "Suspending LVGL display before sleep");
                lv_disp_trig_activity(lv_disp_get_default());
            }
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(0);
            EnableDisplayPower(false); // 关闭显示屏电源
        });
        power_save_timer_->OnExitLightSleepMode([this]() {
            ESP_LOGI(TAG, "Exiting light sleep mode - Wakeup from timer/sleep");
            ESP_LOGI(TAG, "Light sleep exit - Current brightness: %d", GetBacklight()->brightness());
            EnableDisplayPower(true); // 重新开启显示屏电源
            // 添加短暂延迟，让显示屏硬件稳定
            vTaskDelay(pdMS_TO_TICKS(200)); // 增加延时
            // 重新初始化显示使能
            InitializeDisplayEnable();
            // 尝试获取面板句柄并重新初始化显示面板
            if (panel != nullptr) {
                ESP_LOGI(TAG, "Re-initializing AMOLED panel after wake up");
                // 获取面板句柄并重新初始化
                esp_lcd_panel_reset(panel);
                esp_lcd_panel_init(panel);
                esp_lcd_panel_disp_on_off(panel, true);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            // 在唤醒后恢复LVGL显示
            if (lv_disp_get_default() != NULL) {
                ESP_LOGI(TAG, "Resuming LVGL display after wake up");
                lv_disp_trig_activity(lv_disp_get_default());
            }
            auto display = Board::GetInstance().GetDisplay();
            if (display != nullptr) {
                display->UpdateEmojiBoxTime();
            }
            // 恢复LVGL显示活动，避免内存重复释放
            if (lv_disp_get_default() != NULL) {
                ESP_LOGI(TAG, "Resuming LVGL display activity after wake up");
                // 使用安全的API触发显示活动，避免内存重复释放
                lv_disp_trig_activity(lv_disp_get_default());
                // 延迟确保显示稳定
                vTaskDelay(pdMS_TO_TICKS(30));
            }
            GetBacklight()->SetBrightness(20);
            SafeRestartAudioCodec();
            vTaskDelay(pdMS_TO_TICKS(50));
        });
        power_save_timer_->SetEnabled(true);
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

    void InitializeDisplayEnable() {
        // 配置显示屏使能引脚
        gpio_config_t disp_en_config = {};
        disp_en_config.pin_bit_mask = 1ULL << DISPLAY_EN_PIN;
        disp_en_config.mode = GPIO_MODE_OUTPUT;
        disp_en_config.pull_up_en = GPIO_PULLUP_DISABLE;
        disp_en_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        disp_en_config.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&disp_en_config));
        
        // 启用显示屏电源
        ESP_ERROR_CHECK(gpio_set_level(DISPLAY_EN_PIN, 1));
    }
    
    void EnableDisplayPower(bool enable) {
        esp_err_t ret = gpio_set_level(DISPLAY_EN_PIN, enable ? 1 : 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to %s display power", enable ? "enable" : "disable");
        } else {
            ESP_LOGI(TAG, "Display power %s successfully", enable ? "enabled" : "disabled");
        }
    }

    void InitializeMl307Enable() {
        // 配置4G模块使能引脚
        gpio_config_t ml307_en_config = {};
        ml307_en_config.pin_bit_mask = 1ULL << ML307_EN_PIN;
        ml307_en_config.mode = GPIO_MODE_OUTPUT;
        ml307_en_config.pull_up_en = GPIO_PULLUP_DISABLE;
        ml307_en_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        ml307_en_config.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&ml307_en_config));
        
        // 默认启用4G模块
        ESP_ERROR_CHECK(gpio_set_level(ML307_EN_PIN, 1));
        ESP_LOGI(TAG, "ML307 4G module enabled");
    }
    
    void EnableMl307Module(bool enable) {
        esp_err_t ret = gpio_set_level(ML307_EN_PIN, enable ? 1 : 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to %s ML307 4G module", enable ? "enable" : "disable");
        } else {
            ESP_LOGI(TAG, "ML307 4G module %s successfully", enable ? "enabled" : "disabled");
        }
    }

    void CheckNetType() {
        if (GetNetworkType() == NetworkType::WIFI) {
            ESP_LOGI(TAG, "Network type: WiFi, disabling ML307 4G module");
            EnableMl307Module(false);
        } else if (GetNetworkType() == NetworkType::ML307) {
            ESP_LOGI(TAG, "Network type: ML307 4G, enabling 4G module");
            EnableMl307Module(true);
        }
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.sclk_io_num = EXAMPLE_PIN_NUM_LCD_PCLK;
        buscfg.data0_io_num = EXAMPLE_PIN_NUM_LCD_DATA0;
        buscfg.data1_io_num = EXAMPLE_PIN_NUM_LCD_DATA1;
        buscfg.data2_io_num = EXAMPLE_PIN_NUM_LCD_DATA2;
        buscfg.data3_io_num = EXAMPLE_PIN_NUM_LCD_DATA3;
        buscfg.max_transfer_sz = DISPLAY_WIDTH*  DISPLAY_HEIGHT*  sizeof(uint16_t);
        buscfg.flags = SPICOMMON_BUSFLAG_QUAD;
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            // 更新emoji box中的时间显示
            auto display = Board::GetInstance().GetDisplay();
            display->UpdateEmojiBoxTime();
            auto& app = Application::GetInstance();
            // if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                // ResetWifiConfiguration(); 
            // }
            ESP_LOGI(TAG, "Button pressed - Checking wakeup condition: DeviceState=%d (%s), Brightness=%d", app.GetDeviceState(), 
                   app.GetDeviceState() == kDeviceStateIdle ? "kDeviceStateIdle" : "NOT kDeviceStateIdle", 
                   GetBacklight()->brightness());
            if(app.GetDeviceState() == kDeviceStateIdle && GetBacklight()->brightness() == 0){
                GetDisplay()->SetPowerSaveMode(false);
                ESP_LOGI(TAG, "Device waking up from sleep mode");
            }else{
                app.ToggleChatState();
            }
        });

        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting || app.GetDeviceState() == kDeviceStateWifiConfiguring) {
                SwitchNetworkType();
            }
        });
    }

 void InitializeSH8601Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;

        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(
            EXAMPLE_PIN_NUM_LCD_CS,
            nullptr,
            nullptr);
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        const sh8601_vendor_config_t vendor_config = {
            .init_cmds = &vendor_specific_init[0],
            .init_cmds_size = sizeof(vendor_specific_init) / sizeof(sh8601_lcd_init_cmd_t),
            .flags = {
                .use_qspi_interface = 1,
            }};

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        panel_config.vendor_config = (void* )&vendor_config;
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(panel_io, &panel_config, &panel));
        esp_lcd_panel_set_gap(panel, 0x16, 0);
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, false);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_disp_on_off(panel, true);
        display_ = new AmoledDisplay(panel_io, panel,
                                        DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
        backlight_ = new CustomBacklight(panel_io);
        backlight_->RestoreBrightness();
    }

    void InitializeTouch() {
        esp_lcd_touch_handle_t tp;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH - 1,
            .y_max = DISPLAY_HEIGHT - 1,
            .rst_gpio_num = EXAMPLE_PIN_NUM_TOUCH_RST,
            .int_gpio_num = EXAMPLE_PIN_NUM_TOUCH_INT,
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
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST9217_CONFIG();
        tp_io_config.scl_speed_hz = 400*  1000;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle));
        ESP_LOGI(TAG, "Initialize touch controller");
        ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst9217(tp_io_handle, &tp_cfg, &tp));
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(),
            .handle = tp,
        };
        lvgl_port_add_touch(&touch_cfg);
        ESP_LOGI(TAG, "Touch panel initialized successfully");
    }

    void InitializeRtc() {
        // 由于没有RTC芯片，此方法为空
        ESP_LOGW(TAG, "此设备没有配备PCF8563 RTC芯片");
    }

    // 初始化工具
    void InitializeTools() {
        auto &mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.system.reconfigure_wifi",
            "Reboot the device and enter WiFi configuration mode.\n"
            "**CAUTION** You must ask the user to confirm this action.",
            PropertyList(), [this](const PropertyList& properties) {
                // ResetWifiConfiguration(); 
                return true;
            });
    }

public:
    EspWatchAmoled4gBoard() : DualNetworkBoard(ML307_TX_PIN, ML307_RX_PIN, GPIO_NUM_NC),
        boot_button_(BOOT_BUTTON_GPIO, false, 0, 0, true) {
        InitializeI2c();
        InitializeDisplayEnable();
        InitializeMl307Enable();
        CheckNetType();
        InitializePowerSaveTimer();
        InitializePowerManager();
        InitializeSpi();
        InitializeSH8601Display();
        InitializeTouch();
        InitializeButtons();
        InitializeTools();
        InitializeAlarmManager();
        // 由于没有RTC芯片，不初始化RTC
        GetBacklight()->SetBrightness(20);
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
        return backlight_;
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
        DualNetworkBoard::SetPowerSaveMode(enabled);
    }

    virtual SleepTimer* GetPowerSaveTimer() override {
        return power_save_timer_;
    }
};

DECLARE_BOARD(EspWatchAmoled4gBoard);