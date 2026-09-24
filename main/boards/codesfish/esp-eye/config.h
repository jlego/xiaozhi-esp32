#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>
#include <driver/spi_master.h>

#define AUDIO_INPUT_SAMPLE_RATE     16000
#define AUDIO_OUTPUT_SAMPLE_RATE    24000

#define BOOT_BUTTON_GPIO            GPIO_NUM_0
#define POWER_BUTTON_GPIO           GPIO_NUM_45
#define BUILTIN_LED_GPIO            GPIO_NUM_38

//====================MAX98357A 音频芯片
#define AUDIO_SPKR_EN_GPIO          GPIO_NUM_48
#define AUDIO_SPKR_I2S_GPIO_BCLK    GPIO_NUM_18
#define AUDIO_SPKR_I2S_GPIO_LRCLK   GPIO_NUM_16
#define AUDIO_SPKR_I2S_GPIO_DATA    GPIO_NUM_15

//====================ZTS6672 麦克风
#define AUDIO_MIC_I2S_GPIO_BCLK     GPIO_NUM_21
#define AUDIO_MIC_I2S_GPIO_WS       GPIO_NUM_16
#define AUDIO_MIC_I2S_GPIO_DATA     GPIO_NUM_17

#define AUDIO_INPUT_REFERENCE       false
#define VOLUME_UP_BUTTON_GPIO       GPIO_NUM_NC
#define VOLUME_DOWN_BUTTON_GPIO     GPIO_NUM_NC
      
#define I2C_SDA_IO                  GPIO_NUM_1 
#define I2C_SCL_IO                  GPIO_NUM_2        
#define I2C_ADDRESS                 ESP_IO_EXPANDER_I2C_TCA9555_ADDRESS_000

/* Camera pins */
#define CAMERA_PIN_PWDN     GPIO_NUM_12
#define CAMERA_PIN_RESET    GPIO_NUM_43
#define CAMERA_PIN_XCLK     GPIO_NUM_6
#define CAMERA_PIN_SIOD     GPIO_NUM_14
#define CAMERA_PIN_SIOC     GPIO_NUM_13
#define CAMERA_PIN_D9       GPIO_NUM_7
#define CAMERA_PIN_D8       GPIO_NUM_5
#define CAMERA_PIN_D7       GPIO_NUM_4
#define CAMERA_PIN_D6       GPIO_NUM_46
#define CAMERA_PIN_D5       GPIO_NUM_41
#define CAMERA_PIN_D4       GPIO_NUM_40
#define CAMERA_PIN_D3       GPIO_NUM_42
#define CAMERA_PIN_D2       GPIO_NUM_44
#define CAMERA_PIN_VSYNC    GPIO_NUM_11
#define CAMERA_PIN_HREF     GPIO_NUM_39
#define CAMERA_PIN_PCLK     GPIO_NUM_3

#define XCLK_FREQ_HZ 20000000

/* TF card pins */
#define TF_CARD_PIN_EN      GPIO_NUM_47
#define TF_CARD_PIN_CMD     GPIO_NUM_8
#define TF_CARD_PIN_CLK     GPIO_NUM_9
#define TF_CARD_PIN_DTA0    GPIO_NUM_10

#endif // _BOARD_CONFIG_H_