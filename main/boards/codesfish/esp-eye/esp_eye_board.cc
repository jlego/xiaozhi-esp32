#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "sleep_timer.h"
#include "power_manager.h"
#include "mcp_server.h"

#include <esp_log.h>
#include "i2c_device.h"
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/sdmmc_host.h>
#include <driver/sdmmc_types.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <wifi_manager.h>
#include <esp_timer.h>
#include "esp32_camera.h"
#include "jpg/image_to_jpeg.h"
#include "soc/gpio_sig_map.h"

#include <sys/stat.h>
#include <ctime>
#include <cmath>
#include <thread>
#include <atomic>

#include "led/led.h"

#define TAG "esp_eye_board"

extern "C" {
    void esp_rom_gpio_connect_in_signal(uint32_t gpio_num, uint32_t signal_idx, bool inv);
    void esp_rom_gpio_connect_out_signal(uint32_t gpio_num, uint32_t signal_idx, bool out_inv, bool oen_inv);
}

class NoAudioCodecDuplexWithPa : public NoAudioCodecDuplex {
public:
    NoAudioCodecDuplexWithPa(int input_sample_rate, int output_sample_rate,
                             gpio_num_t spk_bclk, gpio_num_t spk_ws, gpio_num_t spk_dout,
                             gpio_num_t mic_sck, gpio_num_t mic_ws, gpio_num_t mic_din,
                             i2s_std_slot_mask_t spk_slot_mask,
                             i2s_std_slot_mask_t mic_slot_mask,
                             bool input_reference,
                             gpio_num_t pa_enable_gpio)
        : NoAudioCodecDuplex(input_sample_rate, output_sample_rate,
                             spk_bclk, spk_ws, spk_dout, mic_din),
          pa_enable_gpio_(pa_enable_gpio),
          spk_bclk_(spk_bclk), spk_ws_(spk_ws),
          mic_bclk_(mic_sck), mic_ws_(mic_ws),
          mic_din_(mic_din) {
        input_reference_ = input_reference;
        input_channels_ = input_reference_ ? 2 : 1;
        input_gain_ = 3.0f;
        duplex_ = true;
        ESP_LOGI(TAG, "Audio codec(Duplex): ref=%d, ch=%d, gain=%.1f",
                 input_reference_, input_channels_, input_gain_);
        ESP_LOGI(TAG, "  SPK: BCLK=%d WS=%d DOUT=%d slot=0x%x",
                 spk_bclk, spk_ws, spk_dout, spk_slot_mask);
        ESP_LOGI(TAG, "  MIC: BCLK=%d WS=%d DIN=%d slot=0x%x",
                 mic_sck, mic_ws, mic_din, mic_slot_mask);

        if (pa_enable_gpio_ != GPIO_NUM_NC) {
            gpio_config_t io_conf = {
                .pin_bit_mask = (1ULL << pa_enable_gpio_),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            ESP_ERROR_CHECK(gpio_config(&io_conf));
            gpio_set_level(pa_enable_gpio_, 0);
        }

        (void)spk_slot_mask;
        (void)mic_slot_mask;
    }

    virtual void Start() override {
        NoAudioCodecDuplex::Start();
        RouteGpiosAfterEnable();
    }

    void RouteGpiosAfterEnable() {
        ESP_LOGI(TAG, "Applying GPIO matrix routes for Duplex shared clock...");

        gpio_num_t primary_ws = (spk_ws_ != I2S_GPIO_UNUSED) ? spk_ws_ : mic_ws_;
        gpio_num_t primary_bclk = (spk_bclk_ != I2S_GPIO_UNUSED) ? spk_bclk_ : mic_bclk_;

        if (primary_bclk != I2S_GPIO_UNUSED && mic_bclk_ != I2S_GPIO_UNUSED && mic_bclk_ != primary_bclk) {
            ESP_LOGI(TAG, "  Route BCLK out: GPIO%d → GPIO%d (mirror)", primary_bclk, mic_bclk_);
            esp_rom_gpio_connect_out_signal(mic_bclk_, I2S0O_BCK_OUT_IDX, false, false);
        }

        if (primary_ws != I2S_GPIO_UNUSED && mic_ws_ != I2S_GPIO_UNUSED && mic_ws_ != primary_ws) {
            ESP_LOGI(TAG, "  Route WS out: GPIO%d → GPIO%d (mirror)", primary_ws, mic_ws_);
            esp_rom_gpio_connect_out_signal(mic_ws_, I2S0O_WS_OUT_IDX, false, false);
        }

        if (mic_din_ != I2S_GPIO_UNUSED) {
            ESP_LOGI(TAG, "  Route MIC DIN: GPIO%d → I2S0I_SD_IN", mic_din_);
            esp_rom_gpio_connect_in_signal(mic_din_, I2S0I_SD_IN_IDX, false);
        }
    }

    virtual void EnableOutput(bool enable) override {
        NoAudioCodecDuplex::EnableOutput(enable);
        if (pa_enable_gpio_ != GPIO_NUM_NC) {
            gpio_set_level(pa_enable_gpio_, enable ? 1 : 0);
            ESP_LOGI(TAG, "PA enable: %s", enable ? "ON" : "OFF");
        }
    }

protected:
    virtual int Read(int16_t* dest, int samples) override {
        size_t bytes_read;
        std::vector<int32_t> bit32_buffer(samples);

        if (i2s_channel_read(rx_handle_, bit32_buffer.data(), samples * sizeof(int32_t), &bytes_read, portMAX_DELAY) != ESP_OK) {
            ESP_LOGE(TAG, "MIC Read Failed!");
            return 0;
        }

        samples = bytes_read / sizeof(int32_t);

        for (int i = 0; i < samples; i++) {
            dc_offset_32_ = dc_offset_32_ * 0.999f + static_cast<float>(bit32_buffer[i]) * 0.001f;
            bit32_buffer[i] -= static_cast<int32_t>(dc_offset_32_);
        }

        for (int i = 0; i < samples; i++) {
            int32_t value = bit32_buffer[i] >> 16;
            dest[i] = (value > INT16_MAX) ? INT16_MAX : (value < -INT16_MAX) ? -INT16_MAX : (int16_t)value;
        }

        if (debug_log_count_++ % 100 == 0) {
            int32_t max_abs = 0;
            int32_t sum = 0;
            for (int i = 0; i < samples; i++) {
                int32_t v = dest[i];
                sum += v;
                if (v < 0) v = -v;
                if (v > max_abs) max_abs = v;
            }
            int32_t avg = sum / samples;
            ESP_LOGI(TAG, "MIC read %d samples: max_abs=%ld, avg=%ld, dc32=%.0f, gain=%.1f",
                     samples, (long)max_abs, (long)avg, dc_offset_32_, input_gain_);
        }

        if (input_gain_ > 0.0f) {
            float gain = input_gain_;
            for (int i = 0; i < samples; i++) {
                int32_t amplified = static_cast<int32_t>(dest[i] * gain);
                if (amplified > INT16_MAX) {
                    dest[i] = INT16_MAX;
                } else if (amplified < -INT16_MAX) {
                    dest[i] = -INT16_MAX;
                } else {
                    dest[i] = static_cast<int16_t>(amplified);
                }
            }
        }
        return samples;
    }

private:
    gpio_num_t pa_enable_gpio_;
    gpio_num_t spk_bclk_, spk_ws_;
    gpio_num_t mic_bclk_, mic_ws_, mic_din_;
    float dc_offset_32_ = 0.0f;
    int debug_log_count_ = 0;
};

#define LED_STANDBY_BRIGHTNESS  8
#define LED_RECORDING_BRIGHTNESS 128
#define LED_PHOTO_BRIGHTNESS    255

#define SDCARD_MOUNT_POINT      "/sdcard"
#define PHOTO_DIR               SDCARD_MOUNT_POINT "/photos"
#define VIDEO_DIR               SDCARD_MOUNT_POINT "/videos"

class EspEyeLed : public Led {
public:
    EspEyeLed(gpio_num_t gpio) : gpio_(gpio) {
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_8_BIT,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = 1000,
            .clk_cfg = LEDC_AUTO_CLK,
            .deconfigure = false
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

        ledc_channel_config_t ledc_channel = {
            .gpio_num = gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags = {
                .output_invert = 0
            }
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                auto self = static_cast<EspEyeLed*>(arg);
                self->OnBlinkTimer();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "led_blink_timer",
            .skip_unhandled_events = false,
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &blink_timer_));
    }

    ~EspEyeLed() {
        esp_timer_stop(blink_timer_);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }

    void TurnOn(uint8_t brightness) {
        esp_timer_stop(blink_timer_);
        blink_active_ = false;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }

    void TurnOff() {
        esp_timer_stop(blink_timer_);
        blink_active_ = false;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }

    void StartBlink(uint8_t brightness, int interval_ms) {
        esp_timer_stop(blink_timer_);
        blink_brightness_ = brightness;
        blink_interval_ms_ = interval_ms;
        blink_active_ = true;
        blink_on_ = true;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        esp_timer_start_periodic(blink_timer_, interval_ms * 1000);
    }

    void OnStateChanged() {
        if (camera_enabled_) {
            StartBlink(LED_RECORDING_BRIGHTNESS, 500);
        } else {
            TurnOn(LED_STANDBY_BRIGHTNESS);
        }
    }

    void SetCameraEnabled(bool enabled) {
        camera_enabled_ = enabled;
        OnStateChanged();
    }

private:
    gpio_num_t gpio_;
    esp_timer_handle_t blink_timer_ = nullptr;
    uint8_t blink_brightness_ = 0;
    int blink_interval_ms_ = 0;
    bool blink_active_ = false;
    bool blink_on_ = true;
    bool camera_enabled_ = false;

    void OnBlinkTimer() {
        blink_on_ = !blink_on_;
        if (blink_on_) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, blink_brightness_);
        } else {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        }
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
};

class CustomBoard : public WifiBoard {
private:
    Button boot_button_;
    Button power_button_;
    i2c_master_bus_handle_t i2c_bus_;
    Esp32Camera* camera_ = nullptr;
    EspEyeLed* esp_eye_led_ = nullptr;
    bool camera_enabled_ = false;
    bool sdcard_mounted_ = false;
    bool video_recording_ = false;
    std::thread video_thread_;
    std::atomic<bool> video_stop_flag_{false};
    camera_config_t camera_config_ = {};
    sdmmc_card_t* sdcard_ = nullptr;

    void InitializeSdCard() {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << TF_CARD_PIN_EN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(TF_CARD_PIN_EN, 1);
    }

    bool MountSdCard() {
        if (sdcard_mounted_) return true;

        gpio_set_level(TF_CARD_PIN_EN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));

        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        host.flags = SDMMC_HOST_FLAG_1BIT | SDMMC_HOST_FLAG_DEINIT_ARG;
        host.max_freq_khz = SDMMC_FREQ_PROBING;

        sdmmc_slot_config_t slot_config = {};
        slot_config.clk = TF_CARD_PIN_CLK;
        slot_config.cmd = TF_CARD_PIN_CMD;
        slot_config.d0 = TF_CARD_PIN_DTA0;
        slot_config.d1 = GPIO_NUM_NC;
        slot_config.d2 = GPIO_NUM_NC;
        slot_config.d3 = GPIO_NUM_NC;
        slot_config.d4 = GPIO_NUM_NC;
        slot_config.d5 = GPIO_NUM_NC;
        slot_config.d6 = GPIO_NUM_NC;
        slot_config.d7 = GPIO_NUM_NC;
        slot_config.cd = SDMMC_SLOT_NO_CD;
        slot_config.wp = SDMMC_SLOT_NO_WP;
        slot_config.width = 1;
        slot_config.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = false,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024,
            .disk_status_check_enable = true,
        };

        esp_err_t ret = esp_vfs_fat_sdmmc_mount(SDCARD_MOUNT_POINT, &host, &slot_config, &mount_config, &sdcard_);
        if (ret == ESP_OK) {
            sdmmc_card_print_info(stdout, sdcard_);
            ESP_LOGI(TAG, "SD card mounted at %s", SDCARD_MOUNT_POINT);
            sdcard_mounted_ = true;
            mkdir(PHOTO_DIR, 0755);
            mkdir(VIDEO_DIR, 0755);
            return true;
        } else {
            ESP_LOGW(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
            gpio_set_level(TF_CARD_PIN_EN, 1);
            return false;
        }
    }

    void UnmountSdCard() {
        if (!sdcard_mounted_) return;

        esp_vfs_fat_sdcard_unmount(SDCARD_MOUNT_POINT, sdcard_);
        sdcard_ = nullptr;
        sdcard_mounted_ = false;
        gpio_set_level(TF_CARD_PIN_EN, 1);
        ESP_LOGI(TAG, "SD card unmounted");
    }

    void InitializeI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = I2C_SDA_IO,
            .scl_io_num = I2C_SCL_IO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiManager::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });

        power_button_.OnClick([this]() {
            ToggleCamera();
        });
    }

    void InitializeCameraConfig() {
        camera_config_ = {
            .pin_pwdn = CAMERA_PIN_PWDN,
            .pin_reset = CAMERA_PIN_RESET,
            .pin_xclk = CAMERA_PIN_XCLK,
            .pin_sccb_sda = CAMERA_PIN_SIOD,
            .pin_sccb_scl = CAMERA_PIN_SIOC,
            .pin_d7 = CAMERA_PIN_D9,
            .pin_d6 = CAMERA_PIN_D8,
            .pin_d5 = CAMERA_PIN_D7,
            .pin_d4 = CAMERA_PIN_D6,
            .pin_d3 = CAMERA_PIN_D5,
            .pin_d2 = CAMERA_PIN_D4,
            .pin_d1 = CAMERA_PIN_D3,
            .pin_d0 = CAMERA_PIN_D2,
            .pin_vsync = CAMERA_PIN_VSYNC,
            .pin_href = CAMERA_PIN_HREF,
            .pin_pclk = CAMERA_PIN_PCLK,

            .xclk_freq_hz = XCLK_FREQ_HZ,
            .ledc_timer = LEDC_TIMER_0,
            .ledc_channel = LEDC_CHANNEL_0,

            .pixel_format = PIXFORMAT_RGB565,
            .frame_size = FRAMESIZE_QVGA,
            .jpeg_quality = 12,
            .fb_count = 2,
            .fb_location = CAMERA_FB_IN_PSRAM,
            .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
            .sccb_i2c_port = (i2c_port_t)1,
        };
    }

    void PowerOnCamera() {
        gpio_set_level(CAMERA_PIN_PWDN, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(CAMERA_PIN_RESET, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(CAMERA_PIN_RESET, 1);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    void PowerOffCamera() {
        gpio_set_level(CAMERA_PIN_PWDN, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    void ToggleCamera() {
        if (!camera_enabled_) {
            ESP_LOGI(TAG, "Powering on camera");
            PowerOnCamera();
            vTaskDelay(pdMS_TO_TICKS(100));
            camera_ = new Esp32Camera(camera_config_);
            if (camera_) {
                camera_->SetHMirror(false);
                camera_enabled_ = true;
                ESP_LOGI(TAG, "Camera powered on");
            } else {
                ESP_LOGE(TAG, "Failed to initialize camera");
            }
        } else {
            ESP_LOGI(TAG, "Powering off camera");
            if (camera_) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
                delete camera_;
#pragma GCC diagnostic pop
                camera_ = nullptr;
            }
            PowerOffCamera();
            camera_enabled_ = false;
            ESP_LOGI(TAG, "Camera powered off");
        }
        if (esp_eye_led_) {
            esp_eye_led_->SetCameraEnabled(camera_enabled_);
        }
    }

    std::string CapturePhoto() {
        if (!camera_enabled_ || !camera_) {
            ESP_LOGE(TAG, "Camera is not enabled");
            return "Camera is not enabled";
        }

        if (!camera_->Capture()) {
            ESP_LOGE(TAG, "Failed to capture frame");
            return "Failed to capture frame";
        }

        if (!MountSdCard()) {
            ESP_LOGE(TAG, "SD card is not available");
            return "SD card is not available";
        }

        std::string result;
        std::vector<uint8_t> jpeg_data;
        QueueHandle_t jpeg_queue = xQueueCreate(40, sizeof(JpegChunk));
        if (jpeg_queue == nullptr) {
            result = "Failed to create JPEG queue";
        } else {
            auto frame_data = const_cast<uint8_t*>(camera_->GetFrameData());
            auto frame_len = camera_->GetFrameLength();
            auto frame_width = camera_->GetFrameWidth();
            auto frame_height = camera_->GetFrameHeight();
            auto frame_format = camera_->GetFrameFormat();

            std::thread encoder_thread([&]() {
                image_to_jpeg_cb(
                    frame_data, frame_len, frame_width, frame_height, frame_format, 80,
                    [](void* arg, size_t index, const void* data, size_t len) -> size_t {
                        auto queue = (QueueHandle_t)arg;
                        JpegChunk chunk = {.data = (uint8_t*)heap_caps_aligned_alloc(16, len, MALLOC_CAP_SPIRAM), .len = len};
                        memcpy(chunk.data, data, len);
                        xQueueSend(queue, &chunk, portMAX_DELAY);
                        return len;
                    },
                    jpeg_queue);
            });

            JpegChunk chunk;
            while (xQueueReceive(jpeg_queue, &chunk, portMAX_DELAY) == pdPASS) {
                if (chunk.data == nullptr) {
                    break;
                }
                jpeg_data.insert(jpeg_data.end(), chunk.data, chunk.data + chunk.len);
                heap_caps_free(chunk.data);
            }

            encoder_thread.join();
            vQueueDelete(jpeg_queue);

            if (jpeg_data.empty()) {
                result = "Failed to encode JPEG";
            } else {
                time_t now = time(nullptr);
                struct tm timeinfo;
                localtime_r(&now, &timeinfo);
                char filename[64];
                snprintf(filename, sizeof(filename), PHOTO_DIR "/photo_%04d%02d%02d_%02d%02d%02d.jpg",
                         timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                         timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

                FILE* f = fopen(filename, "wb");
                if (f == nullptr) {
                    ESP_LOGE(TAG, "Failed to open file: %s", filename);
                    result = "Failed to save photo";
                } else {
                    fwrite(jpeg_data.data(), 1, jpeg_data.size(), f);
                    fclose(f);
                    ESP_LOGI(TAG, "Photo saved: %s (%d bytes)", filename, (int)jpeg_data.size());
                    result = std::string("Photo saved: ") + filename;
                }
            }
        }

        UnmountSdCard();
        return result;
    }

    std::string StartVideoRecording() {
        if (!camera_enabled_ || !camera_) {
            return "Camera is not enabled";
        }
        if (!MountSdCard()) {
            return "SD card is not available";
        }
        if (video_recording_) {
            return "Already recording video";
        }

        video_stop_flag_.store(false);
        video_recording_ = true;

        video_thread_ = std::thread([this]() {
            time_t now = time(nullptr);
            struct tm timeinfo;
            localtime_r(&now, &timeinfo);
            char filename[128];
            snprintf(filename, sizeof(filename), VIDEO_DIR "/video_%04d%02d%02d_%02d%02d%02d.mjpeg",
                     timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

            FILE* f = fopen(filename, "wb");
            if (f == nullptr) {
                ESP_LOGE(TAG, "Failed to open video file: %s", filename);
                video_recording_ = false;
                return;
            }

            int frame_count = 0;
            auto start_time = esp_timer_get_time();

            while (!video_stop_flag_.load()) {
                if (!camera_->Capture()) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }

                auto frame_data = const_cast<uint8_t*>(camera_->GetFrameData());
                auto frame_len = camera_->GetFrameLength();
                auto frame_width = camera_->GetFrameWidth();
                auto frame_height = camera_->GetFrameHeight();
                auto frame_format = camera_->GetFrameFormat();

                QueueHandle_t jpeg_queue = xQueueCreate(40, sizeof(JpegChunk));
                if (jpeg_queue == nullptr) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }

                std::vector<uint8_t> jpeg_data;
                std::thread encoder_thread([&]() {
                    image_to_jpeg_cb(
                        frame_data, frame_len, frame_width, frame_height, frame_format, 75,
                        [](void* arg, size_t index, const void* data, size_t len) -> size_t {
                            auto queue = (QueueHandle_t)arg;
                            JpegChunk chunk = {.data = (uint8_t*)heap_caps_aligned_alloc(16, len, MALLOC_CAP_SPIRAM), .len = len};
                            memcpy(chunk.data, data, len);
                            xQueueSend(queue, &chunk, portMAX_DELAY);
                            return len;
                        },
                        jpeg_queue);
                });

                JpegChunk chunk;
                while (xQueueReceive(jpeg_queue, &chunk, portMAX_DELAY) == pdPASS) {
                    if (chunk.data == nullptr) break;
                    jpeg_data.insert(jpeg_data.end(), chunk.data, chunk.data + chunk.len);
                    heap_caps_free(chunk.data);
                }
                encoder_thread.join();
                vQueueDelete(jpeg_queue);

                if (!jpeg_data.empty()) {
                    fprintf(f, "--frame\r\n");
                    fprintf(f, "Content-Type: image/jpeg\r\n");
                    fprintf(f, "Content-Length: %d\r\n\r\n", (int)jpeg_data.size());
                    fwrite(jpeg_data.data(), 1, jpeg_data.size(), f);
                    fprintf(f, "\r\n");
                    fflush(f);
                    frame_count++;
                }

                int target_interval = std::max(100, (int)(1000 / 5));
                int sleep_ms = target_interval - (int)((esp_timer_get_time() - start_time) / 1000 % target_interval);
                if (sleep_ms > 0) {
                    vTaskDelay(pdMS_TO_TICKS(sleep_ms));
                }
            }

            fclose(f);
            auto duration = (esp_timer_get_time() - start_time) / 1000000;
            ESP_LOGI(TAG, "Video saved: %s (%d frames, %ld seconds)", filename, frame_count, duration);
            video_recording_ = false;
        });

        return "Video recording started";
    }

    std::string StopVideoRecording() {
        if (!video_recording_) {
            return "Not recording video";
        }
        video_stop_flag_.store(true);
        if (video_thread_.joinable()) {
            video_thread_.join();
        }
        UnmountSdCard();
        return "Video recording stopped";
    }

    void InitializeMcpTools() {
        auto& mcp_server = McpServer::GetInstance();

        mcp_server.AddTool("self.camera.enable",
            "开启摄像头，启动录像功能",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (!camera_enabled_) {
                    ToggleCamera();
                }
                return true;
            });

        mcp_server.AddTool("self.camera.disable",
            "关闭摄像头，停止录像",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (camera_enabled_) {
                    ToggleCamera();
                }
                return true;
            });

        mcp_server.AddTool("self.camera.toggle",
            "切换摄像头的开启/关闭状态",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                ToggleCamera();
                return camera_enabled_;
            });

        mcp_server.AddTool("self.get_camera_status",
            "获取摄像头状态，返回是否开启",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return camera_enabled_ ? "enabled" : "disabled";
            });

        mcp_server.AddTool("self.camera.take_photo",
            "拍照并保存照片到TF卡，返回保存的文件路径",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (!camera_enabled_) {
                    ToggleCamera();
                    vTaskDelay(pdMS_TO_TICKS(2000));  // 等待摄像头就绪
                }
                esp_eye_led_->TurnOn(LED_PHOTO_BRIGHTNESS);
                vTaskDelay(pdMS_TO_TICKS(200));
                std::string result = CapturePhoto();
                esp_eye_led_->OnStateChanged();
                return result;
            });

        mcp_server.AddTool("self.camera.start_recording",
            "开始录像，将连续拍摄的画面保存为MJPEG视频到TF卡",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (!camera_enabled_) {
                    ToggleCamera();
                    vTaskDelay(pdMS_TO_TICKS(2000));
                }
                std::string result = StartVideoRecording();
                return result;
            });

        mcp_server.AddTool("self.camera.stop_recording",
            "停止录像，保存视频文件",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                std::string result = StopVideoRecording();
                return result;
            });

        mcp_server.AddTool("self.get_recording_status",
            "获取录像状态，返回是否正在录像",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return video_recording_ ? "recording" : "idle";
            });
    }

public:
    CustomBoard() :
        boot_button_(BOOT_BUTTON_GPIO),
        power_button_(POWER_BUTTON_GPIO) {
        InitializeI2c();
        InitializeButtons();
        InitializeSdCard();
        InitializeCameraConfig();
        InitializeMcpTools();
        gpio_config_t camera_gpio_conf = {
            .pin_bit_mask = (1ULL << CAMERA_PIN_PWDN) | (1ULL << CAMERA_PIN_RESET),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&camera_gpio_conf);
        PowerOffCamera();
    }

    virtual Led* GetLed() override {
        if (!esp_eye_led_) {
            esp_eye_led_ = new EspEyeLed(BUILTIN_LED_GPIO);
        }
        return esp_eye_led_;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecDuplexWithPa audio_codec(
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_SPKR_I2S_GPIO_BCLK,
            AUDIO_SPKR_I2S_GPIO_LRCLK,
            AUDIO_SPKR_I2S_GPIO_DATA,
            AUDIO_MIC_I2S_GPIO_BCLK,
            AUDIO_MIC_I2S_GPIO_WS,
            AUDIO_MIC_I2S_GPIO_DATA,
            I2S_STD_SLOT_LEFT,
            I2S_STD_SLOT_LEFT,
            AUDIO_INPUT_REFERENCE,
            AUDIO_SPKR_EN_GPIO);
        return &audio_codec;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override {
        level = 100;
        charging = false;
        discharging = false;
        return true;
    }

    virtual void SetPowerSaveMode(bool enabled) override {
        WifiBoard::SetPowerSaveMode(enabled);
    }

    virtual SleepTimer* GetPowerSaveTimer() override {
        return nullptr;
    }
};

DECLARE_BOARD(CustomBoard);