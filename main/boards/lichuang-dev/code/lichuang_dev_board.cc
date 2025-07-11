#include "wifi_board.h"
#include "audio_codecs/es8311_audio_codec.h"
#include "display/lcd_display.h"
#include "lcd.h"
#include "application.h"
#include "button.h"
#include "config.h"
// #include "i2c_device.h"  // 注释掉，因为不再使用PCA9557
#include "iot/thing_manager.h"
// #include "esp32_camera.h"  // 注释掉，因为相机功能被注释掉了
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <wifi_station.h>
// #include <esp_lcd_touch_ft5x06.h>  // 注释掉，因为触摸功能被注释掉了
#include <esp_lvgl_port.h>
#include <lvgl.h>

#define TAG "LichuangDevBoard"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);

// class Pca9557 : public I2cDevice {
// public:
//     Pca9557(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
//         WriteReg(0x01, 0x03);
//         WriteReg(0x03, 0xf8);
//     }

//     void SetOutputState(uint8_t bit, uint8_t level) {
//         uint8_t data = ReadReg(0x01);
//         data = (data & ~(1 << bit)) | (level << bit);
//         WriteReg(0x01, data);
//     }
// };

// class CustomAudioCodec : public BoxAudioCodec {
// private:
//     Pca9557* pca9557_;

// public:
//     CustomAudioCodec(i2c_master_bus_handle_t i2c_bus, Pca9557* pca9557) 
//         : BoxAudioCodec(i2c_bus, 
//                        AUDIO_INPUT_SAMPLE_RATE, 
//                        AUDIO_OUTPUT_SAMPLE_RATE,
//                        AUDIO_I2S_GPIO_MCLK, 
//                        AUDIO_I2S_GPIO_BCLK, 
//                        AUDIO_I2S_GPIO_WS, 
//                        AUDIO_I2S_GPIO_DOUT, 
//                        AUDIO_I2S_GPIO_DIN,
//                        GPIO_NUM_NC, 
//                        AUDIO_CODEC_ES8311_ADDR, 
//                        AUDIO_CODEC_ES7210_ADDR, 
//                        AUDIO_INPUT_REFERENCE),
//           pca9557_(pca9557) {
//     }

//     virtual void EnableOutput(bool enable) override {
//         BoxAudioCodec::EnableOutput(enable);
//         if (enable) {
//             pca9557_->SetOutputState(1, 1);
//         } else {
//             pca9557_->SetOutputState(1, 0);
//         }
//     }
// };

// 临时显示类，用于在display_为nullptr时提供安全的显示接口
class TempDisplay : public Display {
public:
    TempDisplay() {
        width_ = 283;  // 横屏模式：宽度283
        height_ = 240; // 横屏模式：高度240
        current_theme_name_ = "light";
    }
    
    virtual void ShowNotification(const char* text, int duration_ms = 3000) override {
        ESP_LOGI("TempDisplay", "Notification: %s (duration: %d ms)", text, duration_ms);
        // 暂时只打印日志，不实际显示
    }
    
    virtual void SetStatus(const char* status) override {
        ESP_LOGI("TempDisplay", "Status: %s", status);
        // 暂时只打印日志，不实际显示
    }
    
    virtual void SetTheme(const std::string& theme_name) override {
        ESP_LOGI("TempDisplay", "Set theme: %s", theme_name.c_str());
        current_theme_name_ = theme_name;
    }
    
    virtual std::string GetTheme() override { 
        return current_theme_name_; 
    }
    
    virtual bool Lock(int timeout_ms = 0) override {
        return true;  // 总是成功
    }
    
    virtual void Unlock() override {
        // 什么都不做
    }
};

// 自定义LCD显示类，使用自定义LCD驱动而不是ESP-IDF面板接口
class CustomLcdDisplay : public LcdDisplay {
public:
    CustomLcdDisplay(DisplayFonts fonts) 
        : LcdDisplay(nullptr, nullptr, fonts, 283, 240) {  // 横屏模式：宽度283，高度240
        // 使用自定义LCD驱动初始化
        ESP_LOGI("CustomLcdDisplay", "Initializing with custom LCD driver");
        
        // 初始化LVGL库
        ESP_LOGI("CustomLcdDisplay", "Initialize LVGL library");
        lv_init();

        // 初始化LVGL端口 - 这是必需的，即使我们使用自定义LCD驱动
        ESP_LOGI("CustomLcdDisplay", "Initialize LVGL port");
        lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
        port_cfg.task_priority = 2;  // 提高任务优先级
        port_cfg.timer_period_ms = 33;  // 降低刷新频率到30Hz，减少系统压力
        lvgl_port_init(&port_cfg);

        // 创建LVGL显示缓冲区
        ESP_LOGI("CustomLcdDisplay", "Creating LVGL display buffer");
        static lv_color_t buf1[283 * 40];  // 横屏模式：宽度283
        static lv_color_t buf2[283 * 40];  // 横屏模式：宽度283

        // 创建LVGL显示，不使用ESP-IDF端口
        ESP_LOGI("CustomLcdDisplay", "Creating LVGL display without ESP-IDF port");
        display_ = lv_display_create(283, 240);  // 横屏模式：宽度283，高度240
        if (display_ == nullptr) {
            ESP_LOGE("CustomLcdDisplay", "Failed to create LVGL display");
            return;
        }

        // 设置显示缓冲区
        lv_display_set_buffers(display_, buf1, buf2, 283 * 40, LV_DISPLAY_RENDER_MODE_PARTIAL);
        
        // 设置颜色格式为RGB565
        lv_display_set_color_format(display_, LV_COLOR_FORMAT_RGB565);
        
        // 设置刷新回调函数（暂时禁用）
        lv_display_set_flush_cb(display_, flush_cb);

        // 现在可以安全地调用SetupUI，因为LVGL端口已经初始化
        ESP_LOGI("CustomLcdDisplay", "Calling SetupUI to create UI interface");
        SetupUI();
        ESP_LOGI("CustomLcdDisplay", "UI interface created successfully");
    }
    
    // LVGL刷新回调函数
    static void flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map) {
        // 设置LCD窗口
        LCD_SetWindows(area->x1, area->y1, area->x2, area->y2);
        LCD_WriteRAM_Prepare();
        
        // 将LVGL缓冲区内容复制到LCD (RGB565格式)
        uint16_t *color_p = (uint16_t *)px_map;
        int width = area->x2 - area->x1 + 1;
        int height = area->y2 - area->y1 + 1;
        int total_pixels = width * height;
        
        // 使用优化的标准传输方法
        const int batch_size = 2048;  // 使用较大的批次大小
        for (int i = 0; i < total_pixels; i += batch_size) {
            int batch_end = (i + batch_size < total_pixels) ? i + batch_size : total_pixels;
            
            // 处理当前批次
            for (int j = i; j < batch_end; j++) {
                Lcd_WriteData_16Bit(color_p[j]);
            }
            
            // 每批处理后让出CPU时间，避免看门狗超时
            if (i + batch_size < total_pixels) {
                vTaskDelay(1);
            }
        }
        
        lv_display_flush_ready(disp);
    }
    
    // 重写绘制函数，使用自定义LCD驱动
    virtual void DrawPixel(int x, int y, uint16_t color) {
        // 使用自定义LCD驱动绘制像素
        LCD_SetCursor(x, y);
        Lcd_WriteData_16Bit(color);
    }
    
    virtual void DrawBuffer(int x, int y, int width, int height, const uint16_t* buffer) {
        // 使用自定义LCD驱动绘制缓冲区
        LCD_SetWindows(x, y, x + width - 1, y + height - 1);
        for (int i = 0; i < width * height; i++) {
            Lcd_WriteData_16Bit(buffer[i]);
        }
    }
    
    // 重写Lock和Unlock方法，使用LVGL端口
    virtual bool Lock(int timeout_ms = 0) override {
        return lvgl_port_lock(timeout_ms);
    }
    
    virtual void Unlock() override {
        lvgl_port_unlock();
    }
    
    // 手动刷新显示方法
    void ManualRefresh() {
        // 手动处理LVGL的刷新
        lv_timer_handler();
    }
};

class LichuangDevBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    // i2c_master_dev_handle_t pca9557_handle_;
    Button boot_button_;
    LcdDisplay* display_;
    TempDisplay temp_display_;  // 临时显示对象
    // Pca9557* pca9557_;
    // Esp32Camera* camera_;  // 注释掉，因为相机功能被注释掉了

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
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

        // Initialize PCA9557 - 注释掉，因为硬件上没有这个芯片
        // pca9557_ = new Pca9557(i2c_bus_, 0x19);
    }

    void InitializeSpi() {
        // 不在这里初始化SPI总线，让自定义LCD驱动自己管理SPI
        // 避免与自定义LCD驱动的SPI初始化冲突
        ESP_LOGI(TAG, "Skipping SPI bus initialization - custom LCD driver will handle it");
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
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
        // 设置ESP-IDF LCD驱动日志级别为ERROR，减少调试输出
        esp_log_level_set("st7789", ESP_LOG_ERROR);
        
        // 使用自定义LCD驱动初始化
        ESP_LOGI(TAG, "Starting LCD initialization...");
        LCD_Init(); // 初始化液晶屏
        ESP_LOGI(TAG, "LCD_Init() completed");

        // 创建自定义LCD显示对象，使用自定义LCD驱动
        ESP_LOGI(TAG, "Creating CustomLcdDisplay with custom LCD driver...");
        display_ = new CustomLcdDisplay({
            .text_font = &font_puhui_20_4,
            .icon_font = &font_awesome_20_4,
#if CONFIG_USE_WECHAT_MESSAGE_STYLE
            .emoji_font = font_emoji_32_init(),
#else
            .emoji_font = font_emoji_64_init(),
#endif
        });
        
        if (display_) {
            ESP_LOGI(TAG, "CustomLcdDisplay created successfully");
        } else {
            ESP_LOGE(TAG, "Failed to create CustomLcdDisplay");
        }
        
        // 添加延迟，确保CustomLcdDisplay创建完成
        vTaskDelay(pdMS_TO_TICKS(100));
    }

public:
    LichuangDevBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();  // 保留I2C初始化，因为音频编解码器可能需要
        InitializeSpi();  // 初始化SPI总线，自定义LCD驱动需要
        InitializeSt7789Display();
        // InitializeTouch();
        InitializeButtons();
        // InitializeCamera();

#if CONFIG_IOT_PROTOCOL_XIAOZHI
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
#endif
        ESP_LOGI(TAG, "Restoring backlight brightness...");
        GetBacklight()->RestoreBrightness();
        ESP_LOGI(TAG, "Backlight restored");
        
        // 强制设置背光为最大亮度
        ESP_LOGI(TAG, "Setting backlight to maximum brightness...");
        GetBacklight()->SetBrightness(100);
        ESP_LOGI(TAG, "Backlight set to maximum brightness");
        
        // 等待背光稳定
        vTaskDelay(pdMS_TO_TICKS(500));
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
        if (display_ == nullptr) {
            ESP_LOGW(TAG, "Display is nullptr, returning temp_display_");
            return &temp_display_;
        }
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual Camera* GetCamera() override {
        return nullptr;  // 返回nullptr，因为相机功能被注释掉了
    }
};

DECLARE_BOARD(LichuangDevBoard);
