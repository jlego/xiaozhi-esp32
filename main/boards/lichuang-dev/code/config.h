#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  24000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

#define AUDIO_INPUT_REFERENCE    true

#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_38
#define AUDIO_I2S_GPIO_WS GPIO_NUM_13
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_14
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_12
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_45

#define AUDIO_CODEC_USE_PCA9557
#define AUDIO_CODEC_PA_PIN       GPIO_NUM_18
#define AUDIO_CODEC_I2C_SDA_PIN  GPIO_NUM_1
#define AUDIO_CODEC_I2C_SCL_PIN  GPIO_NUM_2
#define AUDIO_CODEC_ES8311_ADDR  ES8311_CODEC_DEFAULT_ADDR
// #define AUDIO_CODEC_ES7210_ADDR  0x82  // 注释掉ES7210，因为硬件上没有这个芯片

#define BUILTIN_LED_GPIO        GPIO_NUM_NC
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_NC
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_NC

#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  283
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false

#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0

#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_42
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false

// LCD引脚定义（与lcd.h中的定义保持一致）
// 注意：这些定义在lcd.h中已经存在，这里注释掉避免重复定义
// #define LCD_LED  GPIO_NUM_42
// #define LCD_CS   GPIO_NUM_16
// #define LCD_RS   GPIO_NUM_39
// #define LCD_RST  GPIO_NUM_NC  // 如果硬件上没有复位引脚，设为NC

/* Camera pins */
// #define CAMERA_PIN_PWDN -1
// #define CAMERA_PIN_RESET -1
// #define CAMERA_PIN_XCLK 5
// #define CAMERA_PIN_SIOD 1
// #define CAMERA_PIN_SIOC 2

// #define CAMERA_PIN_D7 9
// #define CAMERA_PIN_D6 4
// #define CAMERA_PIN_D5 6
// #define CAMERA_PIN_D4 15
// #define CAMERA_PIN_D3 17
// #define CAMERA_PIN_D2 8
// #define CAMERA_PIN_D1 18
// #define CAMERA_PIN_D0 16
// #define CAMERA_PIN_VSYNC 3
// #define CAMERA_PIN_HREF 46
// #define CAMERA_PIN_PCLK 7

// #define XCLK_FREQ_HZ 24000000


#endif // _BOARD_CONFIG_H_
